import torch
import torch.nn as nn

from torch.utils.data import DataLoader

from torch_dataset import RobotTrajectoryDataset
from flow_model import FlowMatchingModel

# ==========================================
# Training Configuration
# ==========================================

DATASET_PATH = "data/train_dataset.npz"

BATCH_SIZE = 16
EPOCHS = 300
LEARNING_RATE = 1e-3

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)

# Load dataset
dataset = RobotTrajectoryDataset(DATASET_PATH)

loader = DataLoader(
    dataset,
    batch_size=BATCH_SIZE,
    shuffle=True
)

# MLP Model 285 -> 512 -> 512 -> 192
# Initialize model
model = FlowMatchingModel().to(DEVICE)

# Optimizer
optimizer = torch.optim.Adam(
    model.parameters(),
    lr=LEARNING_RATE
)

# Loss function
criterion = nn.MSELoss()


# ==========================================
# Training Loop
# ==========================================

for epoch in range(EPOCHS):

    model.train()

    total_loss = 0.0

    for batch in loader:

        # Conditions
        start = batch["start"].to(DEVICE)
        goal = batch["goal"].to(DEVICE)
        obstacles = batch["obstacles"].to(DEVICE)
        obstacle_mask = batch["obstacle_mask"].to(DEVICE)

        # Expert trajectory
        z = batch["trajectory"].to(DEVICE)

        batch_size = z.shape[0]

        # Extract intermediate waypoints only
        # (B, 32, 6) -> (B, 30, 6)
        z = z[:, 1:-1, :]

        # Flatten intermediate trajectory
        # (B, 30, 6) -> (B, 180)
        z = z.reshape(batch_size, -1)

        # Sample Gaussian noise
        epsilon = torch.randn_like(z)

        # Sample Flow Matching time
        t = torch.rand(batch_size, 1, device=DEVICE)

        # Construct interpolated state
        x_t = (1 - t) * epsilon + t * z

        # Target velocity
        target_velocity = z - epsilon

        # Predict velocity
        predicted_velocity = model(
            x_t,
            t,
            start,
            goal,
            obstacles,
            obstacle_mask
        )

        # Compute loss
        loss = criterion(
            predicted_velocity,
            target_velocity
        )

        # Backpropagation
        optimizer.zero_grad()

        loss.backward()

        optimizer.step()

        total_loss += loss.item() * batch_size

    average_loss = total_loss / len(dataset)

    if (epoch + 1) % 10 == 0:
        print(
            f"Epoch [{epoch+1}/{EPOCHS}], "
            f"Loss: {average_loss:.6f}"
        )

# ==========================================
# Save Trained Model
# ==========================================

torch.save(
    model.state_dict(),
    "flow_model.pth"
)

print("Training completed!")
print("Model saved to flow_model.pth")