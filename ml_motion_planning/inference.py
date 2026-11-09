import numpy as np
import torch
from pathlib import Path

from torch_dataset import RobotTrajectoryDataset
from flow_model import FlowMatchingModel


# ==========================================
# Inference Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

DATASET_PATH = BASE_DIR / "data/train_dataset.npz"               # provide start, goal, and obstacle data
MODEL_PATH = BASE_DIR / "flow_model.pth"                # trained model weights
OUTPUT_PATH = BASE_DIR / "data/generated_trajectory.npz"         #output path for generated trajectory
  
SAMPLE_INDEX = 0
NUM_STEPS = 30
RANDOM_SEED = 42

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)

# ==========================================
# Load Dataset
# ==========================================

dataset = RobotTrajectoryDataset(DATASET_PATH)

sample = dataset[SAMPLE_INDEX]

start = sample["start"].unsqueeze(0).to(DEVICE)

goal = sample["goal"].unsqueeze(0).to(DEVICE)

obstacles = sample["obstacles"].unsqueeze(0).to(DEVICE)

obstacle_mask = sample["obstacle_mask"].unsqueeze(0).to(DEVICE) # MLP need batch dimension, so we unsqueeze the first dimension

expert_trajectory = sample["trajectory"].numpy()

print("Start shape:", start.shape)
print("Goal shape:", goal.shape)
print("Obstacles shape:", obstacles.shape)

# ==========================================
# Load Trained Model
# ==========================================

model = FlowMatchingModel().to(DEVICE)

model.load_state_dict(
    torch.load(
        MODEL_PATH,
        map_location=DEVICE,
        weights_only=True
    )
)

model.eval()

print("Trained Flow Matching model loaded!")

# ==========================================
# Euler Integration
# ==========================================

torch.manual_seed(RANDOM_SEED)

B = 1

# Generate only 30 intermediate waypoints
TRAJECTORY_DIM = model.trajectory_dim

# Initial Gaussian noise
x = torch.randn(B, TRAJECTORY_DIM, device=DEVICE)

# Euler step size
dt = 1.0 / NUM_STEPS

print("\nStarting Euler Integration...")

with torch.no_grad():

    for step in range(NUM_STEPS):

        # Current Flow Matching time
        t_value = step * dt

        t = torch.full(
            (B, 1),
            t_value,
            device=DEVICE
        )

        # Predict velocity field
        velocity = model(
            x,
            t,
            start,
            goal,
            obstacles,
            obstacle_mask
        )

        # Euler integration
        x = x + dt * velocity

        print(
            f"Step {step+1}/{NUM_STEPS}, "
            f"t = {t_value:.2f}"
        )

# ==========================================
# Convert to Robot Trajectory
# ==========================================

# Intermediate waypoints: (30, 6)
middle_trajectory = x.reshape(30, 6)

# Start and Goal: (1, 6)
start_waypoint = start.reshape(1, 6)
goal_waypoint = goal.reshape(1, 6)

# Reconstruct complete trajectory: (32, 6)
generated_trajectory = torch.cat(
    [
        start_waypoint,
        middle_trajectory,
        goal_waypoint
    ],
    dim=0
)

# Convert to NumPy
generated_trajectory = (
    generated_trajectory
    .cpu()
    .numpy()
)

print("\nGenerated trajectory shape:",
      generated_trajectory.shape)

print("\nGenerated Start:")
print(generated_trajectory[0])

print("\nGenerated Goal:")
print(generated_trajectory[-1])

# ==========================================
# Evaluate Generated Trajectory
# ==========================================

start_np = start[0].cpu().numpy()
goal_np = goal[0].cpu().numpy()

start_error = np.linalg.norm(
    generated_trajectory[0] - start_np
)

goal_error = np.linalg.norm(
    generated_trajectory[-1] - goal_np
)

print("\nStart error:", start_error)
print("Goal error:", goal_error)

# ==========================================
# Save Generated Trajectory
# ==========================================

np.savez_compressed(
    OUTPUT_PATH,
    start=start_np,
    goal=goal_np,
    obstacles=obstacles[0].cpu().numpy(),
    obstacle_mask=obstacle_mask[0].cpu().numpy(),
    generated_trajectory=generated_trajectory,
    expert_trajectory=expert_trajectory
)

print("\nInference completed!")
print("Saved to:", OUTPUT_PATH)