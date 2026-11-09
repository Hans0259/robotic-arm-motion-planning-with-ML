import numpy as np
import rclpy

from pathlib import Path
from rclpy.node import Node

from moveit_msgs.msg import (
    RobotState,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents
)

from moveit_msgs.srv import (
    GetStateValidity,
    ApplyPlanningScene,
    GetPlanningScene
)

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from dobot_planning_scene.random_scene import (
    apply_scene,
    remove_spheres,
)

from dobot_planning_scene.spheres_generator import (
    generate_spheres,
)

from dobot_planning_scene.ground_generator import (
    generate_ground,
)

# ==========================================
# Collision Validation Configuration
# ==========================================

BASE_DIR = Path(__file__).resolve().parent

DATA_PATH = BASE_DIR / "data/generated_trajectory.npz"

GROUP_NAME = "cr10_group"

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6"
]

FRAME_ID = "base_link"

CHECK_STEP = 0.05

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

# ==========================================
# Load Trajectories
# ==========================================

data = np.load(DATA_PATH)

ml_trajectory = data["generated_trajectory"]

rrt_trajectory = data["expert_trajectory"]

obstacles = data["obstacles"]

obstacle_mask = data["obstacle_mask"]

print("ML Trajectory:", ml_trajectory.shape)
print("RRT Trajectory:", rrt_trajectory.shape)

print("Number of obstacles:", int(obstacle_mask.sum()))

# ==========================================
# Joint Limit Validation
# ==========================================

def check_joint_limits(trajectory):

    valid = np.all(
        (trajectory >= JOINT_LOWER) &
        (trajectory <= JOINT_UPPER)
    )

    return bool(valid)


print("\nJoint Limit Validation")

print(
    "RRTConnect:",
    check_joint_limits(rrt_trajectory)
)

print(
    "Flow Matching:",
    check_joint_limits(ml_trajectory)
)

# ==========================================
# MoveIt2 Collision Validator
# ==========================================

class CollisionValidator(Node):

    def __init__(self):
        super().__init__("trajectory_validator")

        # Check robot state validity
        self.validity_client = self.create_client(
            GetStateValidity,
            "/check_state_validity"
        )

        # Apply collision objects
        self.apply_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene"
        )

        # Read current Planning Scene
        self.scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene"
        )

        # Wait for MoveIt2 services
        for client in [
            self.validity_client,
            self.apply_client,
            self.scene_client
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    raise RuntimeError("ROS2 shutdown")

                self.get_logger().info(
                    "Waiting for MoveIt2 services..."
                )

        while not self.validity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for MoveIt2 state validity service..."
            )

    def is_valid(self, joint_values):

        state = RobotState()

        state.joint_state = JointState()
        state.joint_state.name = JOINT_NAMES
        state.joint_state.position = [
            float(q) for q in joint_values
        ]

        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = GROUP_NAME

        future = self.validity_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                "State validity service failed"
            )

        return future.result().valid
    
    def validate_path(self, trajectory):

        total_checked = 0
        invalid_count = 0
        invalid_segments = set()

        for i in range(len(trajectory) - 1):

            q_start = trajectory[i]
            q_end = trajectory[i + 1]

            # Number of interpolation steps
            max_difference = np.max(
                np.abs(q_end - q_start)
            )

            steps = max(
                1,
                int(np.ceil(max_difference / CHECK_STEP))
            )

            for j in range(steps + 1):

                alpha = j / steps

                q = (
                    (1 - alpha) * q_start
                    + alpha * q_end
                )

                total_checked += 1

                if not self.is_valid(q):
                    invalid_count += 1
                    invalid_segments.add(i)

        return {
            "valid": invalid_count == 0,
            "total_checked": total_checked,
            "invalid_count": invalid_count,
            "invalid_segments": sorted(invalid_segments)
        }
    def get_scene_objects(self):

        request = GetPlanningScene.Request()

        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        future = self.scene_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                "Failed to retrieve Planning Scene"
            )

        return future.result().scene.world.collision_objects
    
    def load_obstacles(self, obstacles, obstacle_mask):

        # 1. Read existing scene
        current_objects = self.get_scene_objects()

        # 2. Remove previously generated spheres
        old_ids = [
            obj.id
            for obj in current_objects
            if obj.id.startswith("sphere_")
        ]

        if old_ids:
            remove_spheres(
                self,
                self.apply_client,
                old_ids
            )

            self.get_logger().info(
                f"Removed {len(old_ids)} old spheres"
            )

        # 3. Reconstruct spheres from dataset
        objects = []

        valid_obstacles = obstacles[obstacle_mask == 1]

        for i, obstacle in enumerate(valid_obstacles):

            x, y, z, radius = obstacle

            sphere = generate_spheres(
                float(radius),
                float(x),
                float(y),
                float(z),
                f"sphere_{i}"
            )

            objects.append(sphere)

        # 4. Generate fixed ground
        ground = generate_ground()

        # 5. Apply dataset scene
        apply_scene(
            self,
            self.apply_client,
            [ground] + objects
        )

        self.get_logger().info(
            f"Loaded {len(objects)} dataset spheres + ground"
        )

    def verify_scene(self, obstacles, obstacle_mask):

        current_objects = self.get_scene_objects()

        scene_dict = {
            obj.id: obj
            for obj in current_objects
        }

        expected = obstacles[obstacle_mask == 1]

        expected_ids = {
            "ground",
            *(f"sphere_{i}" for i in range(len(expected)))
        }

        actual_ids = set(scene_dict)

        if actual_ids != expected_ids:
            raise RuntimeError(
                f"Planning Scene IDs mismatch. "
                f"Missing: {expected_ids - actual_ids}; "
                f"Unexpected: {actual_ids - expected_ids}"
            )

        for i, obstacle in enumerate(expected):

            obj = scene_dict[f"sphere_{i}"]

            if len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
                raise RuntimeError(
                    f"sphere_{i} has unexpected geometry"
                )

            primitive = obj.primitives[0]

            if primitive.type != primitive.SPHERE:
                raise RuntimeError(
                    f"sphere_{i} is not a sphere"
                )

            position = obj.pose.position

            print("Object pose:", obj.pose)
            print("Primitive pose:", obj.primitive_poses[0])

            actual = np.array([
                position.x,
                position.y,
                position.z,
                primitive.dimensions[0]
            ])
            print(f"\nSphere {i}")
            print("Frame:", obj.header.frame_id)
            print("Expected:", obstacle)
            print("Actual:  ", actual)

            print("Difference:", actual - obstacle)

            if not np.allclose(
                actual,
                obstacle,
                atol=1e-5,
                rtol=0
            ):
                raise RuntimeError(
                    f"sphere_{i} geometry mismatch"
                )

        # Verify ground dimensions and pose
        ground = scene_dict["ground"]

        if len(ground.primitives) != 1 or len(ground.primitive_poses) != 1:
            raise RuntimeError("Unexpected ground geometry")

        primitive = ground.primitives[0]

        if primitive.type != primitive.BOX:
            raise RuntimeError("Ground is not a box")

        if not np.allclose(
            primitive.dimensions,
            [4.0, 4.0, 0.01],
            atol=1e-5
        ):
            raise RuntimeError("Ground dimensions mismatch")

        position = ground.pose.position

        if not np.allclose(
            [position.x, position.y, position.z],
            [0.0, 0.0, -0.01],
            atol=1e-5
        ):
            raise RuntimeError("Ground position mismatch")

        self.get_logger().info(
            "Planning Scene verified successfully"
        )

# ==========================================
# Main
# ==========================================

def main():

    rclpy.init()

    validator = None

    try:

        validator = CollisionValidator()

        # Restore exact dataset scene
        validator.load_obstacles(
            obstacles,
            obstacle_mask
        )

        # Verify scene geometry
        validator.verify_scene(
            obstacles,
            obstacle_mask
        )

        print("\n========== Collision Validation ==========")

        for name, trajectory in [
            ("RRTConnect", rrt_trajectory),
            ("Flow Matching", ml_trajectory)
        ]:

            result = validator.validate_path(trajectory)

            print(f"\n{name}")
            print("Collision-Free:", result["valid"])
            print("States Checked:", result["total_checked"])
            print("Invalid States:", result["invalid_count"])
            print(
    "Invalid Segments:",
    result["invalid_segments"]
)

    finally:

        if validator is not None:
            validator.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()