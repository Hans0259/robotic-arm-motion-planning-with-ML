"""Collision-check generated trajectories on the validation set with MoveIt.

Requires move_group to be running (same as collect_dataset.py).

For each validation case, K candidate trajectories are generated from
different noise samples and each one is checked with /check_state_validity.

Reported:
    expert valid      sanity check, should be ~100 %
    straight valid    sanity check, should be ~0 % (dataset only keeps blocked cases)
    success@1         first candidate is collision-free
    success@K         at least one of K candidates is collision-free
"""

import argparse
from pathlib import Path

import numpy as np
import rclpy
import torch
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from collect_dataset import Collector, CONFIG, JOINT_LOWER, JOINT_UPPER
from flow_model import FlowMatchingModel, straight_line_middle
from torch_dataset import RobotTrajectoryDataset

# ==========================================
# Evaluation Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

DATASET_PATH = BASE_DIR / "data/val_dataset.npz"
MODEL_PATH = BASE_DIR / "flow_model.pth"
EXPORT_PATH = BASE_DIR / "data/generated_trajectory.npz"

NUM_CASES = 100         # validation samples to evaluate
NUM_CANDIDATES = 8       # K trajectories generated per case
NUM_STEPS = 50           # Euler steps
SEED = 0
EXPORT_CASE = 17         # this case is exported for visualize_trajectory.py （0,1, 4, 9,11,12，15，17）

DEVICE = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)


def generate(model, res_std, start, goal, obstacles, obstacle_mask, k, steps):
    """Generate k candidate trajectories for one case: (k, W, dof) numpy."""
    start = start.unsqueeze(0).expand(k, -1)
    goal = goal.unsqueeze(0).expand(k, -1)
    obstacles = obstacles.unsqueeze(0).expand(k, -1, -1)
    obstacle_mask = obstacle_mask.unsqueeze(0).expand(k, -1)

    x = torch.randn(k, model.trajectory_dim, device=DEVICE)
    dt = 1.0 / steps

    with torch.no_grad():
        for step in range(steps):
            t = torch.full((k, 1), step * dt, device=DEVICE)
            x = x + dt * model(x, t, start, goal, obstacles, obstacle_mask)

    base = straight_line_middle(start, goal, model.num_waypoints)
    middle = base + x.reshape(k, model.num_waypoints - 2, model.dof) * res_std

    trajectory = torch.cat(
        [start.unsqueeze(1), middle, goal.unsqueeze(1)],
        dim=1,
    )
    return trajectory.cpu().numpy().astype(np.float64)


def set_scene(node, obstacles, mask):
    """Replace all sphere_* objects with the spheres stored in the dataset."""
    from dobot_planning_scene.ground_generator import generate_ground
    from dobot_planning_scene.random_scene import apply_scene, remove_spheres
    from dobot_planning_scene.spheres_generator import generate_spheres

    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    existing = node.request(node.scenes, request).scene.world.collision_objects

    old_ids = [obj.id for obj in existing if obj.id.startswith("sphere_")]
    if old_ids:
        remove_spheres(node, node.apply, old_ids)

    spheres = [
        generate_spheres(float(r), float(x), float(y), float(z), f"sphere_{i}")
        for i, (x, y, z, r) in enumerate(obstacles[mask == 1])
    ]
    apply_scene(node, node.apply, [generate_ground()] + spheres)


def trajectory_valid(node, trajectory):
    trajectory = np.asarray(trajectory, dtype=np.float64)
    if not np.isfinite(trajectory).all():
        return False
    if (trajectory < JOINT_LOWER).any() or (trajectory > JOINT_UPPER).any():
        return False
    return node.path_valid(trajectory)


def main():

    torch.manual_seed(SEED)

    # ---- Model ----
    checkpoint = torch.load(MODEL_PATH, map_location=DEVICE, weights_only=True)
    model = FlowMatchingModel(**checkpoint["model_kwargs"]).to(DEVICE)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    res_std = checkpoint["res_std"].to(DEVICE)

    # ---- Data ----
    dataset = RobotTrajectoryDataset(DATASET_PATH)
    num_cases = min(NUM_CASES, len(dataset))

    rclpy.init()
    node = None

    try:
        node = Collector(argparse.Namespace(**CONFIG))

        expert_ok = 0
        straight_ok = 0
        flags = np.zeros((num_cases, NUM_CANDIDATES), dtype=bool)
        scene_key = None

        for i in range(num_cases):
            sample = dataset[i]

            obstacles_np = sample["obstacles"].numpy()
            mask_np = sample["obstacle_mask"].numpy()

            # Only touch the planning scene when the obstacles change
            key = obstacles_np.tobytes() + mask_np.tobytes()
            if key != scene_key:
                set_scene(node, obstacles_np, mask_np)
                scene_key = key

            expert = sample["trajectory"].numpy().astype(np.float64)
            straight = np.linspace(expert[0], expert[-1], len(expert))

            expert_ok += trajectory_valid(node, expert)
            straight_ok += trajectory_valid(node, straight)

            candidates = generate(
                model,
                res_std,
                sample["start"].to(DEVICE),
                sample["goal"].to(DEVICE),
                sample["obstacles"].to(DEVICE),
                sample["obstacle_mask"].to(DEVICE),
                NUM_CANDIDATES,
                NUM_STEPS,
            )

            for k in range(NUM_CANDIDATES):
                flags[i, k] = trajectory_valid(node, candidates[k])

            if i == EXPORT_CASE:
                valid_ids = np.flatnonzero(flags[i])
                chosen = int(valid_ids[0]) if len(valid_ids) else 0
                np.savez_compressed(
                    EXPORT_PATH,
                    start=sample["start"].numpy(),
                    goal=sample["goal"].numpy(),
                    obstacles=obstacles_np,
                    obstacle_mask=mask_np,
                    generated_trajectory=candidates[chosen],
                    expert_trajectory=expert,
                )
                state = "collision-free" if len(valid_ids) else "IN COLLISION"
                print(f"Exported case {i}, candidate {chosen} ({state}) to {EXPORT_PATH}")

            if (i + 1) % 10 == 0:
                done = flags[:i + 1]
                print(
                    f"[{i+1}/{num_cases}]  "
                    f"success@1: {done[:, 0].mean():.1%}  "
                    f"success@{NUM_CANDIDATES}: {done.any(axis=1).mean():.1%}"
                )

        print("\n==== Results ====")
        print(f"Cases evaluated:        {num_cases}")
        print(f"Expert valid:           {expert_ok / num_cases:.1%}   (sanity check, expect ~100%)")
        print(f"Straight line valid:    {straight_ok / num_cases:.1%}   (sanity check, expect ~0%)")
        print(f"success@1:              {flags[:, 0].mean():.1%}")
        print(f"Per-candidate success:  {flags.mean():.1%}")
        print(f"success@{NUM_CANDIDATES}:              {flags.any(axis=1).mean():.1%}")

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()