"""Apply a finite table BOX and verify it in MoveIt's persistent planning scene."""

import math

from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

from ur5_moveit_scripts.real_common import bounded, finite_vector, service_call, wait_until


TABLE_DEFAULTS = {
    'frame_id': 'base_link', 'table_id': 'ur7e_work_surface',
    'size_x': 4.0, 'size_y': 4.0, 'size_z': 0.01,
    'position_x': 0.0, 'position_y': 0.0, 'position_z': -0.010,
}


def table_object(values):
    """Build a horizontal box; positions are its centre, not its top surface."""
    obj = CollisionObject()
    for name in ('frame_id', 'table_id'):
        if not isinstance(values[name], str) or not values[name].strip():
            raise ValueError(f'{name} must be a nonempty string')
    # V1 intentionally limits frames to the fixed, official mounting frames.
    if values['frame_id'] not in ('base_link', 'base', 'world'):
        raise ValueError('frame_id must be base_link, base or world')
    obj.header.frame_id = values['frame_id']
    obj.id = values['table_id']
    box = SolidPrimitive(type=SolidPrimitive.BOX)
    box.dimensions = [bounded(values[f'size_{a}'], 0.001, 20.0, f'size_{a}')
                      for a in 'xyz']
    pose = Pose()
    for axis in 'xyz':
        setattr(pose.position, axis, bounded(
            values[f'position_{axis}'], -20.0, 20.0, f'position_{axis}'))
    pose.orientation.w = 1.0
    obj.primitives = [box]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def get_scene(node):
    """Read geometry, attached objects and collision permissions from MoveIt."""
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    return service_call(node, GetPlanningScene, '/get_planning_scene', request).scene


def require_table(scene, table_id):
    """Require a box with no allowed table collisions or attached objects."""
    if scene.robot_state.attached_collision_objects:
        raise RuntimeError('Attached collision objects are not supported in tool-free V1')
    objects = [o for o in scene.world.collision_objects if o.id == table_id]
    if len(objects) != 1:
        raise RuntimeError(f'Required table {table_id!r} is missing from Planning Scene')
    obj = objects[0]
    if (len(obj.primitives) != 1 or obj.primitives[0].type != SolidPrimitive.BOX
            or len(obj.primitive_poses) != 1):
        raise RuntimeError('Required table must be a finite BOX')
    for value in finite_vector(obj.primitives[0].dimensions, 3, 'table dimensions'):
        bounded(value, 0.001, 20.0, 'table dimension')
    acm = scene.allowed_collision_matrix
    if any(acm.default_entry_values):
        raise RuntimeError('Default allowed collisions found; refusing real planning')
    if table_id in acm.entry_names:
        index = acm.entry_names.index(table_id)
        if any(acm.entry_values[index].enabled) or any(
                row.enabled[index] for row in acm.entry_values):
            raise RuntimeError('Table collisions are allowed in the collision matrix')
    return obj


def apply_table(node, obj):
    """Apply the scene diff, then read it back and verify exact geometry."""
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [obj]
    response = service_call(node, ApplyPlanningScene, '/apply_planning_scene',
                            ApplyPlanningScene.Request(scene=scene), timeout=60.0)
    if not response.success:
        raise RuntimeError('MoveIt rejected the table scene diff')
    accepted = require_table(get_scene(node), obj.id)
    expected_pose = box_pose(obj)
    actual_pose = box_pose(accepted)
    if accepted.header.frame_id != obj.header.frame_id:
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        try:
            target, source = accepted.header.frame_id, obj.header.frame_id
            wait_until(node, lambda: buffer.can_transform(target, source, rclpy.time.Time()),
                       10.0, f'table transform {source} -> {target}')
            expected_pose = do_transform_pose(
                expected_pose, buffer.lookup_transform(target, source, rclpy.time.Time()))
        finally:
            listener.unregister()
    if (accepted.primitives != obj.primitives or not same_pose(expected_pose, actual_pose)):
        raise RuntimeError('Table read-back differs from requested geometry/frame')


def box_pose(obj):
    """Compose object and primitive poses (MoveIt may redistribute these offsets)."""
    transform = TransformStamped()
    transform.transform.translation.x = obj.pose.position.x
    transform.transform.translation.y = obj.pose.position.y
    transform.transform.translation.z = obj.pose.position.z
    transform.transform.rotation = obj.pose.orientation
    return do_transform_pose(obj.primitive_poses[0], transform)


def same_pose(expected, actual):
    """Compare physical poses with floating point tolerance and quaternion sign symmetry."""
    positions_match = all(
        math.isclose(getattr(expected.position, a), getattr(actual.position, a), abs_tol=1e-6)
        for a in 'xyz')
    dot = sum(getattr(expected.orientation, a) * getattr(actual.orientation, a)
              for a in 'xyzw')
    return positions_match and math.isclose(abs(dot), 1.0, abs_tol=1e-6)


def main(args=None):
    """Install the configured table; the object persists after this node exits."""
    rclpy.init(args=args)
    node = Node('add_table_plane')
    try:
        values = {key: node.declare_parameter(key, value).value
                  for key, value in TABLE_DEFAULTS.items()}
        obj = table_object(values)
        apply_table(node, obj)
        node.get_logger().info(
            f'Table VERIFIED: id={obj.id}, frame={obj.header.frame_id}, '
            f'dimensions={obj.primitives[0].dimensions}, '
            f'pose={obj.primitive_poses[0]}. Persists until scene reset/removal.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
