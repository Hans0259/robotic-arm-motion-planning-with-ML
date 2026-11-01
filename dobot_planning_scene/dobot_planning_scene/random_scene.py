import random
import rclpy

from dobot_planning_scene.spheres_generator import generate_spheres
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
    GetStateValidity,
)


def generate_random_spheres(num_spheres=10, random_seed=None):
    random.seed(random_seed)

    spheres = []
    for i in range(num_spheres):
        radius = random.uniform(0.05, 0.2)
        x = random.uniform(-1.3, 1.3)
        y = random.uniform(-0.5, 0.5)
        z = random.uniform(0.1, 0.5)

        object_id = f'sphere_{i}'
        sphere_object = generate_spheres(radius, x, y, z, object_id)
        spheres.append(sphere_object)

    return spheres


# Apply Planning Scene changes
def apply_scene(node, client, objects):
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = objects

    request = ApplyPlanningScene.Request()
    request.scene = scene

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        raise RuntimeError("ApplyPlanningScene service failed")

    if not future.result().success:
        raise RuntimeError("Planning Scene update rejected")


# NEW: Remove spheres by ID
def remove_spheres(node, client, ids):
    objects = []

    for object_id in ids:
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        objects.append(obj)

    if objects:
        apply_scene(node, client, objects)


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('random_scene')

    # Create a client for the PlanningScene service
    client = node.create_client(
        ApplyPlanningScene,
        '/apply_planning_scene'
    )

    # NEW: Clients for current scene and collision checking
    scene_client = node.create_client(
        GetPlanningScene,
        '/get_planning_scene'
    )

    validity_client = node.create_client(
        GetStateValidity,
        '/check_state_validity'
    )

    num_spheres = 5
    random_seed = None

    # Wait for services to be available
    for service_client in [client, scene_client, validity_client]:
        while not service_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
                return
            node.get_logger().info('Waiting for MoveIt2 service...')

    # NEW: Get current robot state and existing objects
    request_scene = GetPlanningScene.Request()
    request_scene.components.components = (
        PlanningSceneComponents.ROBOT_STATE
        | PlanningSceneComponents.WORLD_OBJECT_NAMES
    )

    future = scene_client.call_async(request_scene)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        node.get_logger().error('Failed to get Planning Scene')
        node.destroy_node()
        rclpy.shutdown()
        return

    current_scene = future.result().scene
    robot_state = current_scene.robot_state

    if not robot_state.joint_state.name:
        node.get_logger().error('Robot state is empty')
        node.destroy_node()
        rclpy.shutdown()
        return

    # NEW: Remove previously generated spheres
    old_ids = [
        obj.id
        for obj in current_scene.world.collision_objects
        if obj.id.startswith('sphere_')
    ]

    if old_ids:
        remove_spheres(node, client, old_ids)
        node.get_logger().info(
            f'Removed {len(old_ids)} old spheres'
        )

    # NEW: Helper to check current robot state
    def is_state_valid():
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = 'cr10_group'

        future = validity_client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=5.0
        )

        if not future.done() or future.result() is None:
            raise RuntimeError('Collision check failed')

        return future.result().valid

    # NEW: Check initial robot state
    if not is_state_valid():
        node.get_logger().error(
            'Robot is already in collision before sphere generation'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    # Generate random spheres
    random.seed(random_seed)
    spheres = []

    try:
        for i in range(num_spheres):

            for attempt in range(100):

                # Generate one candidate sphere
                sphere = generate_random_spheres(1, None)[0]
                sphere.id = f'sphere_{i}'

                # Temporarily add candidate
                apply_scene(node, client, [sphere])

                # Check collision with current robot pose
                if is_state_valid():
                    spheres.append(sphere)
                    node.get_logger().info(
                        f'Sphere {i + 1}/{num_spheres} accepted'
                    )
                    break

                # Collision detected: remove and retry
                remove_spheres(node, client, [sphere.id])

            else:
                raise RuntimeError(
                    f'Failed to generate sphere {i + 1}'
                )

        node.get_logger().info(
            'Random spheres generate successfully'
        )

    except Exception as error:
        node.get_logger().error(str(error))

        # Remove candidate and accepted spheres on failure
        remove_spheres(
            node,
            client,
            [f'sphere_{i}' for i in range(num_spheres)]
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()