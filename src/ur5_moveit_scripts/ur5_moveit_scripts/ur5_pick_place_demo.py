"""Visualize a complete table-top pick-and-place cycle in MoveIt and RViz."""

import math
import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node

from ur5_moveit_scripts.motion_common import (
    BackgroundExecutor,
    create_moveit_interface,
    declare_motion_parameters,
    execute_and_report,
    shutdown,
)


TABLE_ID = 'pick_table'
LEG_IDS = [f'pick_table_leg_{index}' for index in range(4)]
OBJECT_ID = 'pick_object'

TABLE_SIZE = (0.70, 0.70, 0.05)
TABLE_POSITION = (-0.55, 0.0, 0.275)
LEG_SIZE = (0.06, 0.06, 0.275)
LEG_POSITIONS = (
    (-0.84, -0.29, 0.1375),
    (-0.84, 0.29, 0.1375),
    (-0.26, -0.29, 0.1375),
    (-0.26, 0.29, 0.1375),
)

OBJECT_HEIGHT = 0.12
OBJECT_RADIUS = 0.035
PICK_OBJECT_POSITION = (-0.45, -0.12, 0.36)
PLACE_OBJECT_POSITION = (-0.45, 0.20, 0.36)

PRE_PICK_POSITION = (-0.45, -0.12, 0.62)
PICK_POSITION = (-0.45, -0.12, 0.48)
LIFT_POSITION = (-0.45, -0.12, 0.68)
PRE_PLACE_POSITION = (-0.45, 0.20, 0.68)
PLACE_POSITION = (-0.45, 0.20, 0.48)
RETREAT_POSITION = (-0.45, 0.20, 0.62)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    """Convert roll, pitch and yaw to an xyzw quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def pause(node: Node, message: str, seconds: float = 1.0) -> None:
    """Pause between visible stages while ROS callbacks continue in the background."""
    node.get_logger().info(message)
    time.sleep(seconds)


def wait_for_controller_active(
    node: Node,
    controller_name: str = 'scaled_joint_trajectory_controller',
    timeout: float = 30.0,
) -> None:
    """Wait until ros2_control reports the trajectory controller as active."""
    client = node.create_client(
        ListControllers,
        '/controller_manager/list_controllers',
    )
    deadline = time.monotonic() + timeout
    last_state = 'not listed'

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if not client.wait_for_service(timeout_sec=min(1.0, remaining)):
                continue

            future = client.call_async(ListControllers.Request())
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)

            if not future.done():
                break

            response = future.result()
            for controller in response.controller:
                if controller.name == controller_name:
                    last_state = controller.state
                    if controller.state == 'active':
                        node.get_logger().info(
                            f'Controller {controller_name} is active.'
                        )
                        return
                    break
            time.sleep(0.25)
    finally:
        node.destroy_client(client)

    raise RuntimeError(
        f'Controller {controller_name} did not become active within '
        f'{timeout:.0f} seconds; last state: {last_state}'
    )


def add_scene(moveit2, node: Node) -> None:
    """Create the table, legs and pick object in the planning scene."""
    moveit2.add_collision_box(
        id=TABLE_ID,
        size=TABLE_SIZE,
        position=TABLE_POSITION,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id='base_link',
    )
    for leg_id, leg_position in zip(LEG_IDS, LEG_POSITIONS):
        moveit2.add_collision_box(
            id=leg_id,
            size=LEG_SIZE,
            position=leg_position,
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id='base_link',
        )
    moveit2.add_collision_cylinder(
        id=OBJECT_ID,
        height=OBJECT_HEIGHT,
        radius=OBJECT_RADIUS,
        position=PICK_OBJECT_POSITION,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id='base_link',
    )
    pause(node, 'Scene ready: table and cylinder are visible in RViz.', 2.0)


def move_to_pose(moveit2, node: Node, label: str, position, orientation) -> None:
    """Plan and execute one named pose stage."""
    node.get_logger().info(f'Stage: {label}; target={position}')
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=orientation,
        frame_id='base_link',
        target_link='tool0',
        tolerance_position=0.01,
        tolerance_orientation=0.02,
        cartesian=False,
    )
    execute_and_report(node, moveit2, label)
    time.sleep(0.8)


def main(args=None):
    """Run the RViz table-top pick-and-place demonstration."""
    rclpy.init(args=args)
    node = Node('ur5_pick_place_demo')
    background = None
    failure = None

    try:
        declare_motion_parameters(node)
        moveit2 = create_moveit_interface(node)
        background = BackgroundExecutor.start(node)
        wait_for_controller_active(node)
        time.sleep(1.0)

        add_scene(moveit2, node)
        downward = quaternion_from_rpy(math.pi, 0.0, math.pi / 2.0)

        move_to_pose(
            moveit2,
            node,
            'approach above object',
            PRE_PICK_POSITION,
            downward,
        )
        move_to_pose(moveit2, node, 'descend to object', PICK_POSITION, downward)

        pause(node, 'Grasp: attaching cylinder to tool0.', 1.0)
        moveit2.attach_collision_object(
            id=OBJECT_ID,
            link_name='tool0',
            touch_links=['tool0', 'flange', 'wrist_3_link'],
        )
        time.sleep(1.5)

        move_to_pose(moveit2, node, 'lift object', LIFT_POSITION, downward)
        move_to_pose(
            moveit2,
            node,
            'transfer above place location',
            PRE_PLACE_POSITION,
            downward,
        )
        move_to_pose(moveit2, node, 'lower object for placement', PLACE_POSITION, downward)

        pause(node, 'Release: detaching cylinder from tool0.', 1.0)
        moveit2.detach_collision_object(OBJECT_ID)
        time.sleep(1.0)
        moveit2.add_collision_cylinder(
            id=OBJECT_ID,
            height=OBJECT_HEIGHT,
            radius=OBJECT_RADIUS,
            position=PLACE_OBJECT_POSITION,
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id='base_link',
        )
        time.sleep(1.0)

        move_to_pose(moveit2, node, 'retreat after placement', RETREAT_POSITION, downward)
        node.get_logger().info(
            'Pick-and-place completed: the cylinder is at the place location.'
        )
    except KeyboardInterrupt:
        node.get_logger().info('Pick-and-place interrupted by user.')
    except Exception as error:  # noqa: BLE001 - clean up ROS before propagating
        node.get_logger().error(f'Pick-and-place failed: {error!r}')
        failure = error
    finally:
        if background is not None:
            shutdown(node, background)
        else:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if failure is not None:
        raise failure


if __name__ == '__main__':
    main()
