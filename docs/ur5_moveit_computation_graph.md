# UR5 机械臂 MoveIt 计算图

> 这份文档对应当前仓库中已经能让 UR5 机械臂运动的代码：`src/ur5_moveit_scripts/`。
> 这里的“计算图”不是深度学习框架里的自动求导图，而是机器人控制/规划系统中的数据流图：目标输入如何经过 Python 节点、MoveIt 规划、控制器执行，最后通过 `/joint_states` 反馈回来。

## 1. 总体计算图

当前实现还不是自己手写的 WBC/QP/RL 低层控制器，而是通过 ROS 2 + `pymoveit2` + MoveIt 2 让机械臂动起来。数据流方向如下：

```mermaid
flowchart LR
    A["Python ROS 2 节点<br/>ur5_pose_goal / ur5_joint_goal<br/><br/>数据类型: rclpy.node.Node"] -->|目标请求<br/>位姿目标: list[float] + list[float]<br/>或关节目标: list[float]| B["pymoveit2 MoveIt2 接口<br/><br/>数据类型: MoveIt2 object"]

    J["/joint_states 反馈<br/><br/>ROS 类型: sensor_msgs/msg/JointState<br/>name: string[]<br/>position: float64[] rad<br/>velocity: float64[] rad/s<br/>effort: float64[]"] -->|当前机器人状态| B

    M["URDF/SRDF + planning scene<br/><br/>数据类型: 机器人模型、碰撞几何、<br/>关节限制、规划组名称"] -->|模型与约束| C["MoveIt 2 规划流水线<br/><br/>功能: IK + 碰撞检测 + 路径规划 + 轨迹生成"]

    B -->|规划/执行 action 请求<br/>数据类型: MoveIt action/service 数据| C

    C -->|如果输入是末端位姿:<br/>IK 求解 q_goal<br/>数据类型: float64[6] rad| D["关节空间规划器<br/>RRTConnect<br/><br/>输入: q_now, q_goal<br/>数据类型: float64[6]"]

    C -->|如果输入是关节目标:<br/>直接使用 q_goal<br/>数据类型: float64[6] rad| D

    D -->|无碰撞路径<br/>数据类型: float64[N,6] rad| E["时间参数化<br/><br/>输出类型: RobotTrajectory"]

    E -->|带时间戳的轨迹<br/>ROS 类型: trajectory_msgs/msg/JointTrajectory<br/>joint_names: string[]<br/>points: JointTrajectoryPoint[]| F["关节轨迹控制器<br/><br/>类型: ros2_control controller"]

    F -->|位置/速度命令<br/>数据类型: float64[6]| G["UR5 mock hardware / 真实驱动<br/><br/>类型: hardware interface"]

    G -->|测量或仿真状态| J
```

核心方向：

- 正向命令链路：Python 脚本 -> `pymoveit2` -> MoveIt 2 -> 关节轨迹控制器 -> mock hardware 或真实机械臂驱动。
- 反向反馈链路：mock hardware 或真实机械臂驱动 -> `/joint_states` -> MoveIt 2 / `pymoveit2`。
- 这个反馈是必要的，因为 MoveIt 必须知道当前关节状态 `q_now`，才能从当前状态规划到目标状态。

## 2. 末端位姿目标计算图

这是 `ur5_pose_goal.py` 中的流程，也是最适合放进周报的计算图，因为它包含了姿态转换、IK、路径规划和轨迹执行。

```mermaid
flowchart TD
    A["TARGET_POSITION<br/>Python 类型: list[float], 长度 3<br/>单位: m<br/>含义: base_link 坐标系下的 [x, y, z]"] --> D["moveit2.move_to_pose(...)"]

    B["TARGET_RPY<br/>Python 类型: list[float], 长度 3<br/>单位: rad<br/>含义: [roll, pitch, yaw]"] --> C["quaternion_from_rpy(roll, pitch, yaw)<br/><br/>计算: sin/cos 三角函数<br/>输入/输出标量类型: float"]

    C -->|quat_xyzw<br/>Python 类型: list[float], 长度 4<br/>顺序: [qx, qy, qz, qw]| D

    P["MoveIt2 配置<br/>joint_names: list[str], 长度 6<br/>base_link_name: str<br/>end_effector_name: str<br/>group_name: str<br/>planner_id: str<br/>max_velocity: float<br/>max_acceleration: float"] --> D

    S["当前关节状态<br/>来源: /joint_states<br/>ROS 类型: sensor_msgs/msg/JointState<br/>position: float64[6] rad"] --> D

    D -->|笛卡尔位姿目标<br/>position: float64[3] m<br/>orientation: float64[4] quaternion<br/>cartesian: bool = False| E["MoveIt IK<br/><br/>输入: 末端位姿<br/>输出: q_goal<br/>数据类型: float64[6] rad"]

    E --> F["RRTConnect 关节空间规划<br/><br/>输入: q_now, q_goal<br/>数据类型: float64[6] rad"]

    F -->|几何路径<br/>数据类型: float64[N,6] rad| G["轨迹生成/时间参数化<br/><br/>输出: positions、velocities、<br/>accelerations、time_from_start"]

    G -->|JointTrajectory<br/>ROS 类型: trajectory_msgs/msg/JointTrajectory| H["控制器执行"]

    H -->|wait_until_executed()<br/>返回: 执行结果/布尔状态| I["Python 节点记录完成信息<br/>然后 shutdown"]
```

对应代码实现：

- `TARGET_POSITION` 是末端执行器目标位置，坐标系是 UR5 的 `base_link`，类型是 `list[float]`，单位是米。
- `TARGET_RPY` 是末端执行器目标姿态，类型是 `list[float]`，单位是弧度。
- `quaternion_from_rpy()` 把 RPY 欧拉角转换成 ROS/MoveIt 使用的四元数顺序 `[qx, qy, qz, qw]`。
- `MoveIt2(...)` 创建规划和执行接口，传入关节名称、基坐标系、末端执行器名称、规划组名称和 ROS callback group。
- `moveit2.move_to_pose(position=..., quat_xyzw=..., cartesian=False)` 把目标末端位姿发送给 MoveIt。
- `cartesian=False` 的含义是：只要求末端最终到达目标位姿，中间路径不强制为笛卡尔直线。MoveIt 会在关节空间中寻找一条可行、无碰撞的路径。
- MoveIt 内部会完成 IK、碰撞检测、RRTConnect 路径规划、时间参数化，然后把 `JointTrajectory` 发给控制器。
- `wait_until_executed()` 等待规划和轨迹执行结束。

## 3. 关节目标计算图

`ur5_joint_goal.py` 和 `ur5_go_home.py` 使用的是关节目标。它比末端位姿目标更简单，因为目标已经是 6 个关节角，不需要 IK。

```mermaid
flowchart TD
    A["SAFE_JOINT_GOAL 或 HOME_JOINTS<br/>Python 类型: list[float], 长度 6<br/>单位: rad<br/>顺序: shoulder_pan, shoulder_lift,<br/>elbow, wrist_1, wrist_2, wrist_3"] --> B["moveit2.move_to_configuration(q_goal)"]

    C["当前关节状态<br/>来源: /joint_states<br/>ROS 类型: sensor_msgs/msg/JointState<br/>position: float64[6] rad"] --> B

    D["MoveIt2 配置<br/>planner_id: str<br/>max_velocity: float<br/>max_acceleration: float"] --> B

    B -->|从 q_now 到 q_goal<br/>数据类型: float64[6] rad| E["RRTConnect 关节空间规划"]

    E -->|路径<br/>数据类型: float64[N,6] rad| F["带时间戳的 JointTrajectory<br/>ROS 类型: trajectory_msgs/msg/JointTrajectory"]

    F --> G["关节轨迹控制器"]

    G --> H["UR5 mock hardware / 真实驱动"]

    H -->|状态反馈| C
```

关节目标链路绕过了 IK，因为 `SAFE_JOINT_GOAL` 和 `HOME_JOINTS` 已经直接给出了 6 个关节角。这种方式适合安全测试位姿和 Home 复位动作。

## 4. 关键数据类型表

| 数据 | 来源 | 类型 | 单位 | 方向 |
|---|---|---|---|---|
| 末端位置 `TARGET_POSITION` | Python 脚本 | `list[float]`, 长度 3 | m | Python -> MoveIt |
| 末端姿态 `TARGET_RPY` | Python 脚本 | `list[float]`, 长度 3 | rad | Python -> 姿态转换函数 |
| 四元数 `quat_xyzw` | `quaternion_from_rpy()` | `list[float]`, 长度 4 | 无量纲 | 姿态转换函数 -> MoveIt |
| 关节目标 `SAFE_JOINT_GOAL` / `HOME_JOINTS` | Python 脚本 | `list[float]`, 长度 6 | rad | Python -> MoveIt |
| 当前关节状态 | `/joint_states` | `sensor_msgs/msg/JointState` | rad, rad/s | 硬件/仿真 -> MoveIt |
| IK 输出 `q_goal` | MoveIt IK | `float64[6]` | rad | IK -> 关节空间规划 |
| 规划路径 | RRTConnect | `float64[N,6]` | rad | 规划器 -> 轨迹生成 |
| 执行轨迹 | MoveIt/控制器接口 | `trajectory_msgs/msg/JointTrajectory` | rad, rad/s, rad/s^2, s | MoveIt -> 控制器 |
| 控制命令 | 关节轨迹控制器 | `float64[6]` 或控制器内部 setpoint | rad / rad/s | 控制器 -> mock hardware/真实驱动 |
| 执行反馈 | mock hardware/真实驱动 | `sensor_msgs/msg/JointState` | rad, rad/s | 硬件/仿真 -> `/joint_states` |

## 5. 为什么这样设计

1. 高层目标比直接发送电流/力矩命令更安全。
   当前阶段的目标是验证机械臂能动起来，因此脚本只给“目标关节角”或“目标末端位姿”，不直接发送电机电流或关节力矩。这样可以利用 MoveIt 和控制器已有的安全检查、限制和轨迹执行机制。

2. MoveIt 负责运动学和规划。
   代码没有手写逆运动学、碰撞检测、路径搜索和轨迹时间参数化。这些任务正是 MoveIt 的职责。脚本只负责定义目标和调用接口，系统复杂度更低，也更容易调试。

3. RPY 必须转成四元数。
   人写目标姿态时 RPY 更直观，但 ROS/MoveIt 的位姿接口使用四元数表示方向。四元数能避免欧拉角的一些奇异性问题，且与 `geometry_msgs/Pose` 的字段顺序一致。

4. `/joint_states` 是闭环反馈入口。
   MoveIt 规划时必须知道当前状态 `q_now`。如果没有当前关节角，就无法确定轨迹起点。因此数据流不是单向命令链路，而是“命令向前、状态向后”的闭环。

5. 末端位姿最终也要变成关节轨迹。
   真实 UR5 的执行器控制的是各个关节，不是直接控制末端坐标。因此即使输入是末端位姿，系统也必须先通过 IK 得到关节目标，再生成 `JointTrajectory`。

6. `cartesian=False` 符合当前测试目标。
   现在主要验证机械臂能从当前状态运动到目标位姿，并不要求末端沿直线移动。使用 `cartesian=False` 可以让 MoveIt 在关节空间中寻找更容易成功的可行路径。

7. 速度和加速度缩放为 0.10 是保守测试策略。
   `max_velocity = 0.10` 和 `max_acceleration = 0.10` 表示使用机器人最大限制的 10%。这适合 mock hardware、RViz 和第一次真实机器人测试，因为动作慢、现象清楚、风险更低。

8. 后台 executor 是必须的。
   `MultiThreadedExecutor` 负责持续处理 ROS topic、action feedback 和执行结果。如果主线程在等待执行结束时没有 executor 处理回调，程序可能收不到反馈，动作状态也无法正确更新。

## 6. 可直接放入周报的文字

本周我实现并测试了基于 ROS 2 + `pymoveit2` + MoveIt 2 的 UR5 机械臂运动。整体计算图从 Python ROS 2 节点开始，输入可以是关节空间目标 `float64[6]`，单位为 rad，也可以是末端位姿目标，其中位置为 `float64[3]`，单位为 m，姿态先用 RPY `float[3]` 表示，再转换为 ROS/MoveIt 使用的四元数 `[qx, qy, qz, qw]`，类型为 `float64[4]`。目标数据通过 `pymoveit2` 发送给 MoveIt 2。MoveIt 从 `/joint_states` 读取当前机器人状态，数据类型为 `sensor_msgs/msg/JointState`。如果输入是末端位姿，MoveIt 先通过 IK 求出目标关节角 `q_goal`；如果输入已经是关节目标，则直接进入关节空间规划。随后 RRTConnect 生成无碰撞路径，MoveIt 将路径时间参数化为 `trajectory_msgs/msg/JointTrajectory`，并发送给关节轨迹控制器执行。控制器驱动 UR5 mock hardware 或真实机械臂驱动，执行后的状态再通过 `/joint_states` 反馈给 MoveIt，从而形成“目标输入 -> 规划 -> 控制执行 -> 状态反馈”的闭环数据流。
