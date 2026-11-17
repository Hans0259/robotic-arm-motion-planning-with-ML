import numpy as np
import torch
from pathlib import Path

from torch_dataset import RobotTrajectoryDataset
from flow_model import FlowMatchingModel, straight_line_middle


# ==========================================
# Inference Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

DATASET_PATH = BASE_DIR / "data/val_dataset.npz"          # unseen start / goal pairs
MODEL_PATH = BASE_DIR / "flow_model.pth"
OUTPUT_PATH = BASE_DIR / "data/generated_trajectory.npz"

SAMPLE_INDEX = 0
NUM_STEPS = 50
RANDOM_SEED = 42          # different seeds give different (possibly all valid) paths

JOINT_LOWER = np.array([-3.14, -3.14, -2.861, -3.14, -3.14, -3.14])
JOINT_UPPER = np.array([3.14, 3.14, 2.861, 3.14, 3.14, 3.14])

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)

# ==========================================
# Load Dataset Sample
# ==========================================

dataset = RobotTrajectoryDataset(DATASET_PATH)
sample = dataset[SAMPLE_INDEX]

start = sample["start"].unsqueeze(0).to(DEVICE)
goal = sample["goal"].unsqueeze(0).to(DEVICE)
obstacles = sample["obstacles"].unsqueeze(0).to(DEVICE)
obstacle_mask = sample["obstacle_mask"].unsqueeze(0).to(DEVICE)

expert_trajectory = sample["trajectory"].numpy()

# ==========================================
# Load Trained Model
# ==========================================

checkpoint = torch.load(MODEL_PATH, map_location=DEVICE, weights_only=True)

model = FlowMatchingModel(**checkpoint["model_kwargs"]).to(DEVICE)
model.load_state_dict(checkpoint["model"])
model.eval()

res_std = checkpoint["res_std"].to(DEVICE)                # (W-2, dof)
num_waypoints = model.num_waypoints
dof = model.dof

print("Trained Flow Matching model loaded!")

# ==========================================
# Euler Integration (in normalized residual space)
# ==========================================

torch.manual_seed(RANDOM_SEED)

x = torch.randn(1, model.trajectory_dim, device=DEVICE)
dt = 1.0 / NUM_STEPS

with torch.no_grad():

    for step in range(NUM_STEPS):

        t = torch.full((1, 1), step * dt, device=DEVICE)

        velocity = model(x, t, start, goal, obstacles, obstacle_mask)

        x = x + dt * velocity

# ==========================================
# Convert back to a Robot Trajectory
# ==========================================

# straight line + de-normalized residual
base = straight_line_middle(start, goal, num_waypoints)   # (1, W-2, dof)
middle = base + x.reshape(1, num_waypoints - 2, dof) * res_std

generated_trajectory = torch.cat(
    [start, middle[0], goal],
    dim=0,
).cpu().numpy()

print("Generated trajectory shape:", generated_trajectory.shape)

# ==========================================
# Evaluate Generated Trajectory
# ==========================================

straight = np.linspace(expert_trajectory[0], expert_trajectory[-1], num_waypoints)


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def path_length(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


print("\n---- Comparison (rad) ----")
print(f"RMSE generated vs expert: {rmse(generated_trajectory, expert_trajectory):.3f}")
print(f"RMSE straight  vs expert: {rmse(straight, expert_trajectory):.3f}  (baseline)")
print(f"Path length  straight: {path_length(straight):.2f}  "
      f"expert: {path_length(expert_trajectory):.2f}  "
      f"generated: {path_length(generated_trajectory):.2f}")
print(f"Largest single-joint jump  expert: "
      f"{np.abs(np.diff(expert_trajectory, axis=0)).max():.3f}  "
      f"generated: {np.abs(np.diff(generated_trajectory, axis=0)).max():.3f}")

within_limits = bool(
    ((generated_trajectory >= JOINT_LOWER) & (generated_trajectory <= JOINT_UPPER)).all()
)
print("Within joint limits:", within_limits)

# ==========================================
# Save Generated Trajectory
# ==========================================

np.savez_compressed(
    OUTPUT_PATH,
    start=start[0].cpu().numpy(),
    goal=goal[0].cpu().numpy(),
    obstacles=obstacles[0].cpu().numpy(),
    obstacle_mask=obstacle_mask[0].cpu().numpy(),
    generated_trajectory=generated_trajectory,
    expert_trajectory=expert_trajectory,
)

print("\nInference completed!")
print("Saved to:", OUTPUT_PATH)