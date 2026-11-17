import copy
from pathlib import Path

import torch

from torch_dataset import RobotTrajectoryDataset
from flow_model import FlowMatchingModel, straight_line_middle

# ==========================================
# Training Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

TRAIN_PATH = BASE_DIR / "data/train_dataset.npz"
VAL_PATH = BASE_DIR / "data/val_dataset.npz"
MODEL_PATH = BASE_DIR / "flow_model.pth"

BATCH_SIZE = 256
TOTAL_STEPS = 50000         # gradient steps; epochs are derived from this
LEARNING_RATE = 3e-4
EMA_DECAY = 0.999
GRAD_CLIP = 1.0
NUM_LOGS = 30               # how many times val loss is evaluated
AUGMENT_REVERSE = True
SEED = 0

MODEL_KWARGS = dict(hidden=2048, emb=128, n_blocks=6)

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)

KEYS = ["start", "goal", "obstacles", "obstacle_mask", "trajectory"]


def load_to_device(path):
    dataset = RobotTrajectoryDataset(path)
    return {key: getattr(dataset, key).to(DEVICE) for key in KEYS}


def add_reversed(data):
    """Augmentation: a collision-free path is also valid when driven backwards."""
    return {
        "start": torch.cat([data["start"], data["goal"]]),
        "goal": torch.cat([data["goal"], data["start"]]),
        "obstacles": torch.cat([data["obstacles"], data["obstacles"]]),
        "obstacle_mask": torch.cat([data["obstacle_mask"], data["obstacle_mask"]]),
        "trajectory": torch.cat([data["trajectory"], data["trajectory"].flip(1)]),
    }


def residual(data):
    """Intermediate waypoints minus the straight line: (N, W-2, dof)."""
    num_waypoints = data["trajectory"].shape[1]
    base = straight_line_middle(data["start"], data["goal"], num_waypoints)
    return data["trajectory"][:, 1:-1] - base


def flow_matching_loss(model, data, z, idx, epsilon, t):
    # Interpolated state and target velocity
    x_t = (1 - t) * epsilon + t * z[idx]
    target_velocity = z[idx] - epsilon

    predicted_velocity = model(
        x_t,
        t,
        data["start"][idx],
        data["goal"][idx],
        data["obstacles"][idx],
        data["obstacle_mask"][idx],
    )

    return torch.mean((predicted_velocity - target_velocity) ** 2)


def main():

    torch.manual_seed(SEED)

    train = load_to_device(TRAIN_PATH)
    if AUGMENT_REVERSE:
        train = add_reversed(train)
    val = load_to_device(VAL_PATH)

    num_train = train["start"].shape[0]
    num_val = val["start"].shape[0]
    num_waypoints = train["trajectory"].shape[1]
    dof = train["trajectory"].shape[2]

    # ------------------------------------------
    # Normalized residual targets
    # ------------------------------------------
    train_residual = residual(train)

    # Per-waypoint, per-joint std (computed on the training set only)
    res_std = train_residual.std(dim=0).clamp(min=1e-3)       # (W-2, dof)

    z_train = (train_residual / res_std).flatten(1)           # (N, 180)
    z_val = (residual(val) / res_std).flatten(1)

    print(f"Device: {DEVICE}")
    print(f"Train samples: {num_train}, Val samples: {num_val}")
    print(f"Residual std: mean {res_std.mean().item():.3f} rad")

    # ------------------------------------------
    # Model, EMA copy, optimizer, schedule
    # ------------------------------------------
    model_kwargs = dict(dof=dof, num_waypoints=num_waypoints, **MODEL_KWARGS)

    model = FlowMatchingModel(**model_kwargs).to(DEVICE)

    ema_model = copy.deepcopy(model)
    ema_model.eval()
    for parameter in ema_model.parameters():
        parameter.requires_grad_(False)

    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=LEARNING_RATE,
        weight_decay=0.0,
    )

    steps_per_epoch = max(1, num_train // BATCH_SIZE)
    EPOCHS = max(1, round(TOTAL_STEPS / steps_per_epoch))
    LOG_EVERY = max(1, EPOCHS // NUM_LOGS)
    total_steps = EPOCHS * steps_per_epoch

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer,
        T_max=total_steps,
    )

    print(f"Total gradient steps: {total_steps}")

    # Fixed noise / time for validation, so the val loss is comparable
    # between epochs instead of jumping around randomly.
    generator = torch.Generator(device=DEVICE).manual_seed(1234)
    val_epsilon = torch.randn(z_val.shape, device=DEVICE, generator=generator)
    val_t = torch.rand(num_val, 1, device=DEVICE, generator=generator)
    val_idx = torch.arange(num_val, device=DEVICE)

    best_val = float("inf")

    # ==========================================
    # Training Loop
    # ==========================================

    for epoch in range(EPOCHS):

        model.train()

        permutation = torch.randperm(num_train, device=DEVICE)
        total_loss = 0.0

        for step in range(steps_per_epoch):

            idx = permutation[step * BATCH_SIZE:(step + 1) * BATCH_SIZE]

            epsilon = torch.randn(len(idx), z_train.shape[1], device=DEVICE)
            t = torch.rand(len(idx), 1, device=DEVICE)

            loss = flow_matching_loss(model, train, z_train, idx, epsilon, t)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRAD_CLIP)
            optimizer.step()
            scheduler.step()

            # Exponential moving average of the weights
            with torch.no_grad():
                for ema_p, p in zip(ema_model.parameters(), model.parameters()):
                    ema_p.mul_(EMA_DECAY).add_(p, alpha=1 - EMA_DECAY)

            total_loss += loss.item()

        if (epoch + 1) % LOG_EVERY == 0 or epoch == 0:

            with torch.no_grad():
                val_loss = flow_matching_loss(
                    ema_model, val, z_val, val_idx, val_epsilon, val_t
                ).item()

            marker = ""
            if val_loss < best_val:
                best_val = val_loss
                torch.save(
                    {
                        "model": ema_model.state_dict(),
                        "model_kwargs": model_kwargs,
                        "res_std": res_std.cpu(),
                    },
                    MODEL_PATH,
                )
                marker = "  <- saved"

            print(
                f"Epoch [{epoch+1}/{EPOCHS}]  "
                f"train loss: {total_loss / steps_per_epoch:.4f}  "
                f"val loss: {val_loss:.4f}{marker}"
            )

    print("Training completed!")
    print(f"Best val loss: {best_val:.4f}")
    print("Model saved to", MODEL_PATH)


if __name__ == "__main__":
    main()