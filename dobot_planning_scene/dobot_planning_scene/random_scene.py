import random
import rclpy
from dobot_planning_scene.ground_generator import generate_ground
from dobot_planning_scene.grid_random_algorithm import generate_grid_spheres
from dobot_planning_scene.spheres_generator import generate_spheres
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)

# generate the x,y,z coordinates and radius of a sphere
def generate_random_spheres(num_spheres=10, random_seed=None):
    random.seed(random_seed)

    spheres = []
    for i in range(num_spheres):
        radius = random.uniform(0.05, 0.2)
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(0.3, 1.0)

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

    # Read existing object IDs so previous spheres can be removed.
    scene_client = node.create_client(
        GetPlanningScene,
        '/get_planning_scene'
    )

    num_spheres = 10
    random_seed = None

    # Wait for services to be available
    for service_client in [client, scene_client]:
        while not service_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
                return
            node.get_logger().info('Waiting for MoveIt2 service...')
    
    # Apply the ground plane to the planning scene
    apply_scene(node, client, [generate_ground()])

    # Scene generation is independent of the current movable-arm posture.
    request_scene = GetPlanningScene.Request()
    request_scene.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES

    future = scene_client.call_async(request_scene)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        node.get_logger().error('Failed to get Planning Scene')
        node.destroy_node()
        rclpy.shutdown()
        return

    current_scene = future.result().scene

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

    try:
        spheres = generate_grid_spheres()
        # spheres = generate_random_spheres(num_spheres=num_spheres, random_seed=random_seed)
        apply_scene(node, client, spheres)
        node.get_logger().info(
            f'Generated {len(spheres)} grid spheres (seed={random_seed})'
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
