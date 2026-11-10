"""Smooth visualization of Flow Matching trajectory in RViz.

RViz:
Displays -> Add -> MoveIt RobotState

Robot State Topic:
/flow_matching/animated_robot

Visualization only. Does not control the robot.
"""

from pathlib import Path
import time

import numpy as np
import rclpy
from moveit_msgs.msg import DisplayRobotState, RobotState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from dobot_planning_scene.spheres_generator import generate_spheres
from dobot_planning_scene.ground_generator import generate_ground
from dobot_planning_scene.random_scene import apply_scene, remove_spheres

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

# --------------------------- Configuration ---------------------------
DATA_PATH = Path(__file__).resolve().parent / "data/generated_trajectory.npz"
ROBOT_STATE_TOPIC = "/flow_matching/animated_robot"
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
SEGMENT_DURATION = 0.4  # seconds between original waypoints
FPS = 30                 # displayed interpolated updates per second
LOOP = False

def load_obstacles(node):

    with np.load(DATA_PATH) as data:
        obstacles = data["obstacles"]
        mask = data["obstacle_mask"]

    apply_client = node.create_client(
        ApplyPlanningScene,
        "/apply_planning_scene"
    )

    scene_client = node.create_client(
        GetPlanningScene,
        "/get_planning_scene"
    )

    for client in [apply_client, scene_client]:
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("ROS2 shutdown")

    # Read existing objects
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
    )

    future = scene_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        raise RuntimeError("Failed to retrieve Planning Scene")

    current_objects = future.result().scene.world.collision_objects

    # Remove old generated spheres
    old_ids = [
        obj.id
        for obj in current_objects
        if obj.id.startswith("sphere_")
    ]

    if old_ids:
        remove_spheres(node, apply_client, old_ids)

    # Reconstruct dataset spheres
    spheres = []

    for i, obstacle in enumerate(obstacles[mask == 1]):
        x, y, z, radius = obstacle

        spheres.append(
            generate_spheres(
                float(radius),
                float(x),
                float(y),
                float(z),
                f"sphere_{i}"
            )
        )

    # Apply ground and spheres
    apply_scene(
        node,
        apply_client,
        [generate_ground()] + spheres
    )

    print(f"Loaded {len(spheres)} spheres + ground")

def load_trajectory():
    with np.load(DATA_PATH) as data:
        trajectory = np.asarray(data["generated_trajectory"], dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != len(JOINT_NAMES) or len(trajectory) < 2:
        raise ValueError(f"Expected trajectory shape (N>=2, 6), got {trajectory.shape}")
    if not np.isfinite(trajectory).all():
        raise ValueError("Trajectory contains NaN or infinite joint angles")
    return trajectory


def publish_pose(node, publisher, q):
    joint_state = JointState()
    joint_state.header.stamp = node.get_clock().now().to_msg()
    joint_state.name = JOINT_NAMES
    joint_state.position = [float(value) for value in q]

    robot_state = RobotState()
    robot_state.joint_state = joint_state
    robot_state.is_diff = False

    display_state = DisplayRobotState()
    display_state.state = robot_state
    publisher.publish(display_state)


def main():
    if SEGMENT_DURATION <= 0 or FPS <= 0:
        raise ValueError("SEGMENT_DURATION and FPS must be positive")

    trajectory = load_trajectory()
    steps_per_segment = max(1, round(SEGMENT_DURATION * FPS))
    frame_period = SEGMENT_DURATION / steps_per_segment
    total_duration = (len(trajectory) - 1) * SEGMENT_DURATION
    print(f"Loaded {trajectory.shape} from {DATA_PATH}")
    print(f"Playback: {total_duration:.1f} s, ~{1/frame_period:.0f} FPS; topic: {ROBOT_STATE_TOPIC}")

    rclpy.init()
    node = rclpy.create_node("flow_matching_rviz_visualizer")
    load_obstacles(node)
    
    publisher = node.create_publisher(
        DisplayRobotState,
        ROBOT_STATE_TOPIC,
        QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    try:
        print("Waiting for RViz RobotState display...")
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        print("RViz connected. Starting smooth playback (Ctrl+C to stop).")

        while rclpy.ok():
            start_clock = time.monotonic()
            total_frames = (len(trajectory) - 1) * steps_per_segment
            for frame in range(total_frames + 1):
                if not rclpy.ok():
                    break
                segment = min(frame // steps_per_segment, len(trajectory) - 2)
                fraction = (frame - segment * steps_per_segment) / steps_per_segment
                fraction = min(1.0, max(0.0, fraction))
                q = (1.0 - fraction) * trajectory[segment] + fraction * trajectory[segment + 1]
                publish_pose(node, publisher, q)

                # Schedule against a monotonic clock to avoid accumulating drift.
                next_time = start_clock + frame * frame_period
                remaining = next_time - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                rclpy.spin_once(node, timeout_sec=0.0)

            if not LOOP:
                break

        if rclpy.ok() and not LOOP:
            print("Playback complete. Final pose remains displayed. Press Ctrl+C to exit.")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()