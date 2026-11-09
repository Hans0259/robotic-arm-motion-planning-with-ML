import numpy as np
import rclpy

from pathlib import Path
from rclpy.node import Node

from moveit_msgs.msg import (
    RobotState,
    CollisionObject,
    PlanningScene
)

from moveit_msgs.srv import (
    GetStateValidity,
    ApplyPlanningScene,
    GetPlanningScene
)

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

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