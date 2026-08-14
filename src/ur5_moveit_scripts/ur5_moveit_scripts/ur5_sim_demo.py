"""Execute one conservative UR5 joint motion in the local mock simulation."""

import rclpy
from rclpy.node import Node

from ur5_moveit_scripts.motion_common import (
    BackgroundExecutor,
    create_moveit_interface,
    declare_motion_parameters,
    execute_and_report,
    finite_float_list,
    shutdown,
)


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
    background = None
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
        background = BackgroundExecutor.start(node)

        # The launch stack is normally already ready. This short delay also
        # allows the first /joint_states sample to reach pymoveit2.
        node.create_rate(1.0).sleep()

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
