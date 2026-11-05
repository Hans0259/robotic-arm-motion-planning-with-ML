import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path

# ==========================================
# Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

DATA_PATH = BASE_DIR / "data/generated_trajectory.npz"
OUTPUT_PATH = BASE_DIR / "data/trajectory_comparison.png"

# ==========================================
# Load Trajectories
# ==========================================

data = np.load(DATA_PATH)

ml_trajectory = data["generated_trajectory"]
rrt_trajectory = data["expert_trajectory"]

start = data["start"]
goal = data["goal"]

waypoints = np.arange(len(ml_trajectory))

# ==========================================
# Plot Joint Trajectories
# ==========================================

fig, axes = plt.subplots(3, 2, figsize=(12, 10))

for i, ax in enumerate(axes.flat):

    ax.plot(
        waypoints,
        rrt_trajectory[:, i],
        label="RRTConnect",
        linewidth=2
    )

    ax.plot(
        waypoints,
        ml_trajectory[:, i],
        label="Flow Matching",
        linestyle="--",
        linewidth=2
    )

    ax.scatter(
        [0, len(waypoints) - 1],
        [start[i], goal[i]],
        marker="x",
        s=70,
        label="Target Start/Goal"
    )

    ax.set_title(f"Joint {i+1}")

    ax.set_xlabel("Waypoint Index")

    ax.set_ylabel("Joint Angle (rad)")

    ax.grid(True)

    ax.legend()

plt.suptitle(
    "RRTConnect vs Flow Matching Trajectory",
    fontsize=14
)

plt.tight_layout()

plt.savefig(
    OUTPUT_PATH,
    dpi=300
)

plt.show()