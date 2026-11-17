"""Compare statistics of real vs generated trajectories on the validation set.

Same idea as Task 5 of the course Flow Matching notebook (real vs generated
mean / std), applied to trajectory residuals. Needs PyTorch only, not MoveIt.

If the model learned the trajectory distribution, generated statistics should
be close to the real ones. If "Generated std" is much smaller than "Real std",
the samples are collapsing toward the straight line.
"""

from pathlib import Path

import numpy as np
import torch

from flow_model import FlowMatchingModel, straight_line_middle
from torch_dataset import RobotTrajectoryDataset

BASE_DIR = Path(__file__).resolve().parent

DATASET_PATH = BASE_DIR / "data/val_dataset.npz"
MODEL_PATH = BASE_DIR / "flow_model.pth"

NUM_STEPS = 50
SEED = 0

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)


def path_length(trajectory):
    # (N, W, dof) -> (N,)
    return np.linalg.norm(np.diff(trajectory, axis=1), axis=2).sum(axis=1)


def main():

    torch.manual_seed(SEED)

    checkpoint = torch.load(MODEL_PATH, map_location=DEVICE, weights_only=True)
    model = FlowMatchingModel(**checkpoint["model_kwargs"]).to(DEVICE)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    res_std = checkpoint["res_std"].to(DEVICE)               # (W-2, dof)

    dataset = RobotTrajectoryDataset(DATASET_PATH)
    start = dataset.start.to(DEVICE)
    goal = dataset.goal.to(DEVICE)
    obstacles = dataset.obstacles.to(DEVICE)
    obstacle_mask = dataset.obstacle_mask.to(DEVICE)
    expert = dataset.trajectory.to(DEVICE)

    n = start.shape[0]
    waypoints, dof = model.num_waypoints, model.dof

    # ---- Generate one trajectory per validation case ----
    x = torch.randn(n, model.trajectory_dim, device=DEVICE)
    dt = 1.0 / NUM_STEPS

    with torch.no_grad():
        for step in range(NUM_STEPS):
            t = torch.full((n, 1), step * dt, device=DEVICE)
            x = x + dt * model(x, t, start, goal, obstacles, obstacle_mask)

    base = straight_line_middle(start, goal, waypoints)       # (N, W-2, dof)

    # Normalized residuals: real data has std ~1 in this space by construction
    real_z = ((expert[:, 1:-1] - base) / res_std).cpu().numpy()
    generated_z = x.reshape(n, waypoints - 2, dof).cpu().numpy()

    print(f"Cases: {n}")
    print("\nNormalized residual statistics per joint")
    print("Joint | Real Mean | Generated Mean | Real Std | Generated Std")
    print("-" * 62)
    for j in range(dof):
        print(
            f"{j + 1:^5} | "
            f"{real_z[:, :, j].mean():^9.3f} | "
            f"{generated_z[:, :, j].mean():^14.3f} | "
            f"{real_z[:, :, j].std():^8.3f} | "
            f"{generated_z[:, :, j].std():^13.3f}"
        )
    print(
        f"{'all':^5} | "
        f"{real_z.mean():^9.3f} | {generated_z.mean():^14.3f} | "
        f"{real_z.std():^8.3f} | {generated_z.std():^13.3f}"
    )

    # ---- Path length relative to the straight line ----
    middle = base + x.reshape(n, waypoints - 2, dof) * res_std
    generated = torch.cat(
        [start.unsqueeze(1), middle, goal.unsqueeze(1)], dim=1
    ).cpu().numpy()
    expert_np = expert.cpu().numpy()

    straight_length = np.linalg.norm(
        (goal - start).cpu().numpy(), axis=1
    )
    expert_ratio = path_length(expert_np) / straight_length
    generated_ratio = path_length(generated) / straight_length

    print("\nDetour ratio (path length / straight-line length)")
    print(f"Expert:    mean {expert_ratio.mean():.3f}, median {np.median(expert_ratio):.3f}")
    print(f"Generated: mean {generated_ratio.mean():.3f}, median {np.median(generated_ratio):.3f}")

    # ---- How far from the straight line, in radians ----
    real_dev = np.abs(expert_np[:, 1:-1] - base.cpu().numpy()).max(axis=(1, 2))
    generated_dev = np.abs(generated[:, 1:-1] - base.cpu().numpy()).max(axis=(1, 2))

    print("\nLargest deviation from the straight line per trajectory [rad]")
    print(f"Expert:    mean {real_dev.mean():.3f}")
    print(f"Generated: mean {generated_dev.mean():.3f}")


if __name__ == "__main__":
    main()