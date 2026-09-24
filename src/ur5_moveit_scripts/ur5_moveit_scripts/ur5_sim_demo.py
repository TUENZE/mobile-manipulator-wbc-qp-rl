"""Execute one conservative UR5 joint motion in the local mock simulation."""

import time

from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.node import Node

from ur5_moveit_scripts.motion_common import (
    create_moveit_interface,
    declare_motion_parameters,
    execute_and_report,
    finite_float_list,
)
from ur5_moveit_scripts.real_common import service_call, wait_until


DEFAULT_JOINT_GOAL = [
    0.0,
    -1.5707963268,
    1.5707963268,
    -1.5707963268,
    -1.5707963268,
    0.0,
]


def main(args=None):
    """Run the parameterized, simulation-safe motion demo."""
    rclpy.init(args=args)
    node = Node('ur5_sim_demo')
    failure = None

    try:
        declare_motion_parameters(node)
        node.declare_parameter('joint_goal', DEFAULT_JOINT_GOAL)
        joint_goal = finite_float_list(
            node.get_parameter('joint_goal').value,
            length=6,
            name='joint_goal',
        )

        moveit2 = create_moveit_interface(node)
        # Installed Jazzy pymoveit2 plan()/wait_until_executed() spin internally.
        # A second executor on this same node races the ROS action wait set.
        wait_until(node, lambda: moveit2.joint_state is not None, 15.0, 'mock joint states')
        planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        try:
            if not planner.wait_for_service(timeout_sec=60.0):
                raise TimeoutError('Mock MoveIt planning service is unavailable')
        finally:
            node.destroy_client(planner)
        deadline = time.monotonic() + 30.0
        while True:
            controllers = service_call(
                node, ListControllers, '/controller_manager/list_controllers',
                ListControllers.Request())
            if any(c.name == 'scaled_joint_trajectory_controller' and c.state == 'active'
                   for c in controllers.controller):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Mock trajectory controller did not become active')
            rclpy.spin_once(node, timeout_sec=0.2)

        node.get_logger().info(
            'Simulation-only UR5 demo: mock hardware, end effector=tool0, '
            f'joint_goal={joint_goal}'
        )
        moveit2.move_to_configuration(joint_goal)
        execute_and_report(node, moveit2, 'UR5 mock joint trajectory')
    except KeyboardInterrupt:
        node.get_logger().info('Simulation demo interrupted by user.')
    except Exception as error:  # noqa: BLE001 - preserve ROS cleanup before raising
        node.get_logger().error(f'Simulation demo failed: {error!r}')
        failure = error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failure is not None:
        raise failure


if __name__ == '__main__':
    main()
