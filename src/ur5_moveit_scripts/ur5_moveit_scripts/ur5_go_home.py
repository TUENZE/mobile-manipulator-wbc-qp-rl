#!/usr/bin/env python3
"""
Move the UR5 robot back to its natural startup / Home configuration.

This Home configuration was recorded from /joint_states immediately
after the UR5 MoveIt simulation was started.
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as ur_robot


# ============================================================
# UR5 natural startup / Home configuration
#
# Joint order required by pymoveit2:
# 1. shoulder_pan_joint
# 2. shoulder_lift_joint
# 3. elbow_joint
# 4. wrist_1_joint
# 5. wrist_2_joint
# 6. wrist_3_joint
#
# Unit: radians
# ============================================================
HOME_JOINTS = [
    0.0,    # shoulder_pan_joint
    -1.57,  # shoulder_lift_joint
    0.0,    # elbow_joint
    -1.57,  # wrist_1_joint
    0.0,    # wrist_2_joint
    0.0,    # wrist_3_joint
]


def main():
    rclpy.init()

    # Create this ROS 2 node.
    node = Node("ur5_go_home")

    # Allow MoveIt callbacks to run in parallel.
    callback_group = ReentrantCallbackGroup()

    # Create the pymoveit2 control interface for UR5.
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur_robot.joint_names(),
        base_link_name=ur_robot.base_link_name(),
        end_effector_name=ur_robot.end_effector_name(),
        group_name=ur_robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Use the same planner that you used in your previous scripts.
    moveit2.planner_id = "RRTConnectkConfigDefault"

    # Spin ROS callbacks in a background thread.
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    # Give ROS / MoveIt a moment to finish initializing.
    node.create_rate(1.0).sleep()

    try:
        # 0.10 means 10% of the robot's allowed maximum values.
        # Keeping it slow makes the Home motion easier to observe and safer.
        moveit2.max_velocity = 0.10
        moveit2.max_acceleration = 0.10

        node.get_logger().info(
            "Sending UR5 to natural startup / Home configuration:"
        )
        node.get_logger().info(str(HOME_JOINTS))

        # Plan and execute motion from the current position to Home.
        moveit2.move_to_configuration(HOME_JOINTS)

        # Keep this script alive until trajectory execution has finished.
        execution_success = moveit2.wait_until_executed()

        if execution_success:
            node.get_logger().info(
                "UR5 successfully returned to the Home configuration."
            )
        else:
            node.get_logger().error(
                "UR5 did not report successful Home-position execution."
            )

    except KeyboardInterrupt:
        node.get_logger().warn(
            "UR5 Home motion was stopped by the user."
        )

    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()