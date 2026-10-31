import numpy as np
from dataset import create_sample, save_dataset

# Example start and goal
start = np.array([0, -0.5, 0.3, 0, 0.7, 0])

goal = np.array([0.5, -0.2, 0.6, 0.3, 0.4, 0.1])

# Example obstacles
obstacles = [
    [0.4, 0.2, 0.3, 0.08],
    [0.5, 0.1, 0.4, 0.06]
]

# Dummy trajectory for testing data format
trajectory = np.linspace(start, goal, 32)

# Create sample
sample = create_sample(
    start,
    goal,
    obstacles,
    trajectory
)

# Save
save_dataset(
    [sample],
    "data/test_dataset.npz"
)

# Check result
data = np.load("data/test_dataset.npz")

for key in data.files:
    print(key, data[key].shape)