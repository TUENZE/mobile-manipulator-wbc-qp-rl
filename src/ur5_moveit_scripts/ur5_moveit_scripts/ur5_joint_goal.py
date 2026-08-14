from threading import Thread

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as ur_robot

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


# 单位是 rad（弧度）
# 顺序必须与 UR5 的六个关节顺序一致：
# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
SAFE_JOINT_GOAL = [
    0.0,
    -1.57,
    1.57,
    -1.57,
    -1.57,
    0.0,
]


def main(args=None):
    rclpy.init(args=args)

    node = Node('ur5_joint_goal')
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur_robot.joint_names(),
        base_link_name=ur_robot.base_link_name(),
        end_effector_name=ur_robot.end_effector_name(),
        group_name=ur_robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # 与你刚才在 RViz 中实际看到的 OMPL / RRTConnect 对应
    moveit2.planner_id = 'RRTConnectkConfigDefault'

    # 设置为最大速度、最大加速度的 10%
    # 目前是 mock hardware；养成低速测试习惯
    moveit2.max_velocity = 0.10
    moveit2.max_acceleration = 0.10

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        # 给 action/service client 一点初始化时间
        node.create_rate(1.0).sleep()

        node.get_logger().info(
            f'Sending UR5 joint goal: {SAFE_JOINT_GOAL}'
        )

        # 这一步会请求 MoveIt：
        # 当前 joint state -> SAFE_JOINT_GOAL
        # 自动规划轨迹，然后发送给 controller 执行
        moveit2.move_to_configuration(SAFE_JOINT_GOAL)

        # 等待轨迹执行结束
        moveit2.wait_until_executed()

        node.get_logger().info(
            'UR5 trajectory execution finished.'
        )

    except KeyboardInterrupt:
        node.get_logger().info('Execution interrupted by user.')

    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()
