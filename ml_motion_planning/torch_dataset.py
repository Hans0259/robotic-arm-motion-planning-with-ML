import numpy as np
import torch

from torch.utils.data import Dataset, DataLoader

class RobotTrajectoryDataset(Dataset):

    def __init__(self, file_path):

        # Load NPZ dataset
        data = np.load(file_path)

        # Robot configurations
        self.start = torch.tensor(
            data["start"], dtype=torch.float32
        )

        self.goal = torch.tensor(
            data["goal"], dtype=torch.float32
        )

        # Obstacles
        self.obstacles = torch.tensor(
            data["obstacles"], dtype=torch.float32
        )

        self.obstacle_mask = torch.tensor(
            data["obstacle_mask"], dtype=torch.float32
        )

        # Expert trajectories
        self.trajectory = torch.tensor(
            data["trajectory"], dtype=torch.float32
        )

    def __len__(self):
        return len(self.start)

    def __getitem__(self, idx):
        return {
            "start": self.start[idx],
            "goal": self.goal[idx],
            "obstacles": self.obstacles[idx],
            "obstacle_mask": self.obstacle_mask[idx],
            "trajectory": self.trajectory[idx],
        }

if __name__ == "__main__":

    # Load dataset
    dataset = RobotTrajectoryDataset(
        "data/train_dataset.npz"
    )

    print("Total samples:", len(dataset))

    # Test a single sample
    sample = dataset[0]

    print("\nSingle Sample:")

    for key, value in sample.items():
        print(key, value.shape)

    # Create DataLoader
    loader = DataLoader(
        dataset,
        batch_size=8,
        shuffle=True
    )

    # Test one batch
    batch = next(iter(loader))

    print("\nMini-Batch:")

    for key, value in batch.items():
        print(key, value.shape)