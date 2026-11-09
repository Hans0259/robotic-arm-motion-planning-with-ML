import random

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def generate_ground():
    thickness = 0.01

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [4.0, 4.0, thickness]

    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.z = -0.01

    ground = CollisionObject()
    ground.header.frame_id = 'base_link'
    ground.id = 'ground'
    ground.primitives = [primitive]
    ground.primitive_poses = [pose]
    ground.operation = CollisionObject.ADD

    return ground