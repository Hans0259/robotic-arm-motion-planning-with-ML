import argparse
import random
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import (
    Constraints, JointConstraint, PlanningSceneComponents,
    RobotState, MoveItErrorCodes,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetStateValidity,
)
from sensor_msgs.msg import JointState

from dataset import DOF, NUM_WAYPOINTS, MAX_OBSTACLES, create_sample, save_dataset

from dobot_planning_scene.random_scene import apply_scene,remove_spheres
from dobot_planning_scene.grid_random_algorithm import generate_grid_spheres
from dobot_planning_scene.ground_generator import generate_ground

# ==========================================
# Dataset Collection Configuration
# ==========================================

CONFIG = {
    # Robot
    "group": "cr10_group",
    "joints": [
        "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6"
    ],
    "frame": "base_link",

    # Motion Planning
    "planner_id": "RRTConnectkConfigDefault",
    "planning_time": 5.0,

    # Dataset Collection
    "samples": 2000,
    "max_attempts": 10000,
    "obstacles": 15,

    # Validation
    "goal_tolerance": 0.01,
    "check_step": 0.05,

    # Random Seed
    "seed": 42,
    "scene_seed": 123,

    # Dataset Output
    "output": "data/train_dataset.npz"
}


# Joint position limits from MoveIt2 URDF
JOINT_LOWER = np.array([
    -3.14,
    -3.14,
    -2.861,
    -3.14,
    -3.14,
    -3.14
])

JOINT_UPPER = np.array([
    3.14,
    3.14,
    2.861,
    3.14,
    3.14,
    3.14
])

def resample_joint_path(path, count=NUM_WAYPOINTS):
    """Interpolate by cumulative joint-space distance, not trajectory time."""
    path = np.asarray(path, dtype=np.float64)
    if path.ndim != 2 or path.shape[1] != DOF or len(path) < 2:
        raise ValueError('Path must have at least two 6-DOF waypoints')
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    keep = np.r_[True, np.diff(cumulative) > 1e-10]
    path, cumulative = path[keep], cumulative[keep]
    if len(path) < 2:
        raise ValueError('Degenerate path')
    targets = np.linspace(0.0, cumulative[-1], count)
    output = np.column_stack([
        np.interp(targets, cumulative, path[:, col]) for col in range(DOF)
    ])
    return output.astype(np.float32)


class Collector(Node):
    def __init__(self, args):
        super().__init__('ml_dataset_collector')
        self.args = args
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scenes = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        for name, client in [('plan', self.planner), ('scene', self.scenes),
                             ('apply', self.apply), ('validity', self.validity)]:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f'Missing MoveIt service ({name}). Start move_group first.')
        self.rng = np.random.default_rng(args.seed)
        self.previous_ids = []

    def request(self, client, request, timeout=30):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service call failed or timed out: {future.exception() if future.done() else "timeout"}')
        return future.result()

    def current_state(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE
        state = self.request(self.scenes, req).scene.robot_state
        by_name = dict(zip(state.joint_state.name, state.joint_state.position))
        missing = [j for j in self.args.joints if j not in by_name]
        if missing:
            raise RuntimeError(f'Joint(s) absent from planning scene: {missing}')
        return np.array([by_name[j] for j in self.args.joints], dtype=float)

    def joint_state(self, angles):
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(self.args.joints)
        state.joint_state.position = [float(x) for x in angles]
        state.is_diff = True  # preserve non-group joints in MoveIt current state
        return state

    def is_valid(self, angles):
        req = GetStateValidity.Request()
        req.robot_state = self.joint_state(angles)
        req.group_name = self.args.group
        return bool(self.request(self.validity, req).valid)

    def sample_valid_configuration(self, max_trials=100):

        for _ in range(max_trials):

            q = self.rng.uniform(
                JOINT_LOWER,
                JOINT_UPPER
            )

            if self.is_valid(q):
                return q

        return None

    def set_obstacles(self):

        # Remove last iteration's spheres before creating the next scene.
        if self.previous_ids:
            remove_spheres(self, self.apply, self.previous_ids)
            self.previous_ids = []

        seed = self.args.scene_seed

        objects = generate_grid_spheres(
            num_spheres=self.args.obstacles,
            random_seed=seed
        )

        encoded = []
        for obj in objects:
            # Keep IDs from Harry's generator (sphere_0, sphere_1, ...).
            # The helper creates CollisionObject messages, not a live scene.
            if obj.header.frame_id != self.args.frame:
                raise RuntimeError(
                    f'Sphere frame {obj.header.frame_id!r} does not match '
                    f'--frame {self.args.frame!r}'
                )
            if not obj.primitives or not obj.primitive_poses:
                raise RuntimeError('Sphere CollisionObject lacks primitive/pose')
            p = obj.primitive_poses[0].position
            r = float(obj.primitives[0].dimensions[0])
            encoded.append([p.x, p.y, p.z, r])

        # Generate fixed ground
        ground = generate_ground()

        # Apply ground and spheres
        apply_scene(
            self,
            self.apply,
            [ground] + objects
        )  

        self.previous_ids = [obj.id for obj in objects]
        return encoded

    def plan(self, start, goal):
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name = self.args.group
        motion.planner_id = self.args.planner_id
        motion.allowed_planning_time = self.args.planning_time
        motion.num_planning_attempts = 1
        motion.start_state = self.joint_state(start)
        constraints = Constraints()
        for name, angle in zip(self.args.joints, goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(angle)
            jc.tolerance_above = self.args.goal_tolerance
            jc.tolerance_below = self.args.goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        motion.goal_constraints = [constraints]
        response = self.request(self.planner, req, timeout=self.args.planning_time + 15)
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        trajectory = result.trajectory.joint_trajectory
        names = list(trajectory.joint_names)
        if any(j not in names for j in self.args.joints):
            raise RuntimeError(f'Planner returned unexpected joints: {names}')
        indices = [names.index(j) for j in self.args.joints]
        return np.array([[pt.positions[k] for k in indices]
                         for pt in trajectory.points], dtype=float)

    def path_valid(self, waypoints):
        # Check additional interpolated configurations between stored waypoints.
        for a, b in zip(waypoints[:-1], waypoints[1:]):
            steps = max(1, int(np.ceil(np.max(np.abs(b - a)) / self.args.check_step)))
            for t in np.linspace(0, 1, steps + 1):
                if not self.is_valid(a + t * (b - a)):
                    return False
        return True
    
    def straight_path_valid(self, start, goal):

        straight = np.linspace(start, goal, NUM_WAYPOINTS)

        return self.path_valid(straight)

    def collect(self):
        samples = []
        attempts = 0

        # Generate obstacles only once
        obs = self.set_obstacles()

        while len(samples) < self.args.samples and attempts < self.args.max_attempts:
            attempts += 1
            try:
                obs = self.set_obstacles()
                # Generate random valid Start
                start = self.sample_valid_configuration()

                if start is None:
                    continue

                # Generate random valid Goal
                goal = self.sample_valid_configuration()

                if goal is None:
                    continue

                # Check whether straight-line motion is valid
                straight_valid = self.straight_path_valid(start, goal)

                if straight_valid:
                    self.get_logger().info(
                        'Straight path is valid; skipping'
                    )
                    continue
                # Straight path is blocked: try RRTConnect
                raw = self.plan(start, goal)
                if raw is None or len(raw) < 2:
                    continue
                sampled = resample_joint_path(raw)
                if not self.path_valid(sampled):
                    continue
                # The actual final point may differ slightly due to goal tolerance.
                samples.append(create_sample(start, raw[-1], obs, sampled))
                self.get_logger().info(f'Accepted {len(samples)}/{self.args.samples} (attempt {attempts})')
            except (RuntimeError, ValueError) as exc:
                self.get_logger().warning(f'Attempt {attempts} skipped: {exc}')
        if samples:
            path = Path(self.args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            save_dataset(samples, str(path))
        else:
            self.get_logger().error('No valid samples collected; no file saved')
        self.get_logger().info(f'Accepted {len(samples)} samples out of {attempts} attempts')


def main():

    # Load configuration
    args = argparse.Namespace(**CONFIG)

    # Validate configuration
    if not 0 <= args.obstacles <= MAX_OBSTACLES:
        raise ValueError(
            f"Number of obstacles must be between 0 and {MAX_OBSTACLES}"
        )

    if len(args.joints) != DOF:
        raise ValueError(
            f"Expected {DOF} joint names"
        )

    # Initialize ROS2
    rclpy.init()

    node = None

    try:
        node = Collector(args)
        node.collect()

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()