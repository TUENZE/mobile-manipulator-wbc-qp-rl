#!/usr/bin/env python3

import math
from threading import Thread

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as ur_robot

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


# ---------------------------------------------------------
# 目标末端位姿：相对于 UR5 的 base_link 坐标系
#
# 位置单位：m
# 姿态单位：rad
#
# 这是一个适合 mock hardware 初次测试的保守目标：
# 末端大致朝下，并位于机械臂前下方可达区域。
#
# 注意：
# 这不是“任何真实 UR5 环境下都绝对安全”的通用位置。
# 第一次只在 mock hardware / RViz 中测试。
# ---------------------------------------------------------
TARGET_POSITION = [
    -0.45,  # x: 前后方向（相对于 base_link）
    -0.11,  # y: 左右方向
    0.48,   # z: 高度
]

TARGET_RPY = [
    math.pi,       # roll:  绕 X 轴旋转 180°
    0.0,           # pitch: 绕 Y 轴旋转 0°
    math.pi / 2,   # yaw:   绕 Z 轴旋转 90°
]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    """
    Convert Roll-Pitch-Yaw Euler angles to a ROS quaternion.

    将 Roll-Pitch-Yaw（Euler angles）转换为 ROS / MoveIt 使用的四元数。

    返回顺序必须是：
    [qx, qy, qz, qw]
    而不是 [qw, qx, qy, qz]。
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    node = Node('ur5_pose_goal')

    # 允许 MoveIt 的 Action、joint state 等回调被多线程处理
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur_robot.joint_names(),
        base_link_name=ur_robot.base_link_name(),
        end_effector_name=ur_robot.end_effector_name(),
        group_name=ur_robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # 与你在 RViz 中使用的 OMPL / RRTConnect 对应
    moveit2.planner_id = 'RRTConnectkConfigDefault'

    # 初次测试时保持低速
    # 这是相对于 robot joint limits 最大值的缩放比例
    moveit2.max_velocity = 0.10
    moveit2.max_acceleration = 0.10

    # 后台持续处理 ROS topic、Action feedback、执行结果等
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        # 给 MoveIt、Action client、joint states 一点初始化时间
        node.create_rate(1.0).sleep()

        roll, pitch, yaw = TARGET_RPY
        target_quat_xyzw = quaternion_from_rpy(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

        node.get_logger().info(
            'Sending UR5 pose goal:\n'
            f'  position [m]  = {TARGET_POSITION}\n'
            f'  RPY [rad]     = {TARGET_RPY}\n'
            f'  quat [x,y,z,w]= {target_quat_xyzw}'
        )

        # 你给的是 Cartesian pose：
        # position + orientation
        #
        # MoveIt 内部会：
        # 1. 用 IK 寻找能到达该 pose 的 joint configuration；
        # 2. 用 RRTConnect 等 planner 在 joint space 规划轨迹；
        # 3. 将生成的 trajectory 发送给 controller 执行。
        #
        # cartesian=False 的含义：
        # “末端最终到达这个 pose”。
        # 中间末端路线不保证是一条直线。
        moveit2.move_to_pose(
            position=TARGET_POSITION,
            quat_xyzw=target_quat_xyzw,
            cartesian=False,
        )

        # 等待规划和轨迹执行结束
        moveit2.wait_until_executed()

        node.get_logger().info(
            'UR5 pose trajectory execution finished.'
        )

    except KeyboardInterrupt:
        node.get_logger().info(
            'Execution interrupted by user.'
        )

    except Exception as error:
        node.get_logger().error(
            f'Pose motion failed: {error!r}'
        )

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()
