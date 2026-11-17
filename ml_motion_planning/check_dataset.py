"""Sanity checks for data/train_dataset.npz.

Usage:
    python3 check_dataset.py        # run all checks
    python3 check_dataset.py 9      # also export expert sample 9 for RViz playback
                                    # (then run: python3 visualize_trajectory.py)
"""

import sys
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
DATASET_PATH = BASE_DIR / "data/train_dataset.npz"
EXPORT_PATH = BASE_DIR / "data/generated_trajectory.npz"

JOINT_LOWER = np.array([-3.14, -3.14, -2.861, -3.14, -3.14, -3.14])
JOINT_UPPER = np.array([3.14, 3.14, 2.861, 3.14, 3.14, 3.14])


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


def main():
    data = np.load(DATASET_PATH)
    start = data["start"]
    goal = data["goal"]
    obstacles = data["obstacles"]
    mask = data["obstacle_mask"]
    traj = data["trajectory"]
    n = len(start)

    print("---- Shapes ----")
    for key in data.files:
        print(f"{key:15s} {data[key].shape}")

    print("\n---- Hard checks (should all PASS) ----")
    check("No NaN / inf",
          all(np.isfinite(data[k]).all() for k in data.files))
    check("Shapes consistent",
          start.shape == (n, 6) and goal.shape == (n, 6)
          and traj.ndim == 3 and traj.shape[0] == n and traj.shape[2] == 6
          and obstacles.shape[:2] == mask.shape and obstacles.shape[2] == 4)

    err_s = np.abs(traj[:, 0] - start).max()
    err_g = np.abs(traj[:, -1] - goal).max()
    check("trajectory[0] == start", err_s < 1e-4, f"(max err {err_s:.2e})")
    check("trajectory[-1] == goal", err_g < 1e-4, f"(max err {err_g:.2e})")

    in_limits = ((traj >= JOINT_LOWER - 1e-3) & (traj <= JOINT_UPPER + 1e-3)).all()
    check("All waypoints within joint limits", in_limits)

    counts = np.unique(mask.sum(axis=1))
    check("Same obstacle count in every sample", len(counts) == 1,
          f"(counts: {counts.tolist()})")
    check("Fixed scene: obstacles identical across samples",
          np.allclose(obstacles, obstacles[0]))

    pairs = np.round(np.hstack([start, goal]), 4)
    n_unique = len(np.unique(pairs, axis=0))
    check("No duplicated (start, goal) pairs", n_unique == n,
          f"({n_unique}/{n} unique)")

    print("\n---- Quality statistics (for your judgement) ----")
    steps = np.linalg.norm(np.diff(traj, axis=1), axis=2)      # (N, W-1)
    length = steps.sum(axis=1)
    straight = np.linalg.norm(goal - start, axis=1)
    ratio = length / np.maximum(straight, 1e-9)

    print(f"Straight-line distance [rad]: "
          f"min {straight.min():.2f}, median {np.median(straight):.2f}, max {straight.max():.2f}")
    print(f"Path length [rad]:            "
          f"min {length.min():.2f}, median {np.median(length):.2f}, max {length.max():.2f}")
    print(f"Detour ratio (path/straight): "
          f"min {ratio.min():.2f}, median {np.median(ratio):.2f}, max {ratio.max():.2f}")
    print(f"Samples with ratio > 2: {(ratio > 2).sum()}/{n},  > 3: {(ratio > 3).sum()}/{n}")
    print(f"Largest single-joint jump between waypoints [rad]: "
          f"{np.abs(np.diff(traj, axis=1)).max():.3f}")

    mid = traj[:, 1:-1]
    w = np.linspace(0, 1, traj.shape[1])[1:-1].reshape(1, -1, 1)
    base = start[:, None, :] * (1 - w) + goal[:, None, :] * w
    residual = mid - base
    print(f"Residual from straight line: std {residual.std():.3f} rad "
          f"(this is RES_STD for training)")

    worst = np.argsort(ratio)[::-1][:5]
    print("Most detoured samples (index: ratio):",
          ", ".join(f"{i}: {ratio[i]:.2f}" for i in worst))

    if len(sys.argv) > 1:
        idx = int(sys.argv[1])
        np.savez_compressed(
            EXPORT_PATH,
            start=start[idx],
            goal=goal[idx],
            obstacles=obstacles[idx],
            obstacle_mask=mask[idx],
            generated_trajectory=traj[idx],
            expert_trajectory=traj[idx],
        )
        print(f"\nExported expert sample {idx} to {EXPORT_PATH}")
        print("Run: python3 visualize_trajectory.py")


if __name__ == "__main__":
    main()