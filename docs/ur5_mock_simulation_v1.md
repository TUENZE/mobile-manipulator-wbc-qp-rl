# UR5 本机纯仿真第一版

## 范围

这一版提供可复现的 UR5 运动规划与控制基线：

- ROS 2 Jazzy + MoveIt 2；
- Universal Robots 官方 UR5 URDF、SRDF、关节限制与控制器配置；
- ros2_control 的 `mock_components/GenericSystem`；
- OMPL 默认 RRTConnect 规划；
- RViz 可视化；
- 可选的一键关节轨迹演示。

这是运动学/控制接口仿真，并包含基于 MoveIt Planning Scene attach/detach 的桌面抓取可视化；不包含 Gazebo 刚体动力学、真实接触或传感器噪声。它适合先验证 ROS 计算图、MoveIt 规划和控制器执行链路。后续 WBC、QP、RL 或真实抓取实验应在这个稳定基线上分阶段增加。

## 硬件隔离

项目 launch 固定使用：

- `use_mock_hardware=true`；
- `robot_ip=127.0.0.1`；
- `launch_dashboard_client=false`；
- `use_tool_communication=false`；
- `headless_mode=true`。

项目没有真实机器人 IP 配置，也不会启动 Dashboard、URScript 或真实 robot state helper 通信路径。

## 末端执行器

第一版使用 Universal Robots 官方模型和 `pymoveit2` 的默认末端工具坐标系 `tool0`。这是库存 UR5 MoveIt 配置中最兼容、最少假设的默认 TCP；当前不附加第三方夹爪几何和控制器。

需要物体抓取时，建议下一版单独集成 Robotiq 2F-85，并同步增加 URDF/SRDF、碰撞矩阵、夹爪 ros2_control 控制器和 MoveIt planning group。不要只改 `end_effector_name`，否则模型、碰撞与控制器会不一致。

## 构建

```bash
cd /home/tuenze/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur5_moveit_scripts --symlink-install
source install/setup.bash
```

## 启动

只启动仿真与 MoveIt，手动在 RViz 中规划：

```bash
ros2 launch ur5_moveit_scripts ur5_mock_moveit.launch.py
```

无界面启动并自动执行一次保守关节运动：

```bash
ros2 launch ur5_moveit_scripts ur5_mock_moveit.launch.py \
  launch_rviz:=false run_demo:=true
```

自动动作默认关闭，只有明确设置 `run_demo:=true` 才会在启动约 8 秒后执行。

也可以先启动仿真，再在另一个已 source 的终端运行：

```bash
ros2 run ur5_moveit_scripts ur5_sim_demo \
  --ros-args --params-file \
  "$(ros2 pkg prefix ur5_moveit_scripts)/share/ur5_moveit_scripts/config/motion_defaults.yaml"
```

## RViz 桌面抓取与放置演示

终端 1 启动 mock hardware、MoveIt 和 RViz：

```bash
cd /home/tuenze/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5_moveit_scripts ur5_mock_moveit.launch.py launch_rviz:=true run_demo:=false
```

保持终端 1 运行，在终端 2 中启动桌面抓取演示：

```bash
cd /home/tuenze/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_moveit_scripts ur5_pick_place_demo
```

第二条命令启动后，RViz 会显示桌面、四条桌腿和一个圆柱。UR5 将依次靠近、下降、附着圆柱、抬升、搬运、放下、释放并撤离。当前默认末端是 `tool0`，因此抓取由 MoveIt Planning Scene 的 attach/detach 表示，不包含夹爪手指开合或 Gazebo 接触动力学。

## 参数

默认参数位于 `src/ur5_moveit_scripts/config/motion_defaults.yaml`：

- `max_velocity: 0.10`：最大关节速度的 10%；
- `max_acceleration: 0.10`：最大关节加速度的 10%；
- `joint_goal`：6 个关节目标，单位 rad；
- `position`：末端位置，单位 m；
- `rpy`：末端姿态欧拉角，单位 rad。

演示节点会检查数组长度、有限数值以及速度/加速度缩放范围；MoveIt 执行失败时节点以异常结束，不会误报成功。

## 验证命令

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select ur5_moveit_scripts
colcon test-result --verbose
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

MoveIt 启动时关于未配置 Octomap 3D 传感器的日志，是当前无相机第一版的预期提示，不影响关节规划与执行。普通用户中止长期运行的 launch 后，个别上游节点也可能在退出阶段记录超时；验收应以轨迹执行的 `SUCCEEDED` 和演示节点的成功退出为准。
