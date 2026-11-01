import numpy as np

# Dataset configuration
DOF = 6
NUM_WAYPOINTS = 32
MAX_OBSTACLES = 20


def create_sample(start, goal, obstacles, trajectory):

    start = np.asarray(start, dtype=np.float32)
    goal = np.asarray(goal, dtype=np.float32)
    obstacles = np.asarray(obstacles, dtype=np.float32).reshape(-1, 4)
    trajectory = np.asarray(trajectory, dtype=np.float32)

    # Validate dimensions
    assert start.shape == (DOF,)
    assert goal.shape == (DOF,)
    assert trajectory.shape == (NUM_WAYPOINTS, DOF)

    assert len(obstacles) <= MAX_OBSTACLES

    # Padding for obstacles
    padded_obstacles = np.zeros(
        (MAX_OBSTACLES, 4),
        dtype=np.float32
    )

    obstacle_mask = np.zeros(
        MAX_OBSTACLES,
        dtype=np.float32
    )

    n = len(obstacles)

    padded_obstacles[:n] = obstacles
    obstacle_mask[:n] = 1.0

    return {
        "start": start,
        "goal": goal,
        "obstacles": padded_obstacles,
        "obstacle_mask": obstacle_mask,
        "trajectory": trajectory
    }


def save_dataset(samples, filename):

    if not samples:
        raise ValueError("No samples to save")

    dataset = {}

    for key in samples[0]:
        dataset[key] = np.stack(
            [sample[key] for sample in samples]
        )

    np.savez_compressed(filename, **dataset)

    print(f"Saved {len(samples)} samples to {filename}")