# UR5 本机纯仿真第一版

## 范围

这一版提供可复现的 UR5 运动规划与控制基线：

- ROS 2 Jazzy + MoveIt 2；
- Universal Robots 官方 UR5 URDF、SRDF、关节限制与控制器配置；
- ros2_control 的 `mock_components/GenericSystem`；
- OMPL 默认 RRTConnect 规划；
- RViz 可视化；
- 安装在 `tool0` 上的通用平行两指夹爪；
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

当前模型在 Universal Robots 官方 UR5 的 `tool0` 下安装了一个通用平行两指夹爪。夹爪包含掌部、两个 prismatic 手指关节、内侧防滑垫以及位于抓取中心的 `gripper_tcp`。

夹爪采用纯运动学状态节点控制：张开位置为 `0.065 m`，闭合位置为 `0.045 m`。闭合时两个黑色指垫的内表面分别位于抓取中心 `±0.035 m`，与演示圆柱的 `0.035 m` 半径一致，因此 RViz 中能看到两指贴合圆柱。UR5 六轴仍由 MoveIt 和 mock ros2_control 执行，夹爪不依赖 Gazebo。

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

第二条命令启动后，RViz 会显示桌面、四条桌腿和一个圆柱。UR5 将依次靠近、下降、闭合两个手指、将圆柱附着到 `gripper_tcp`、抬升、搬运、放下、张开手指、释放并撤离。

轨迹模式按阶段区分：从初始状态到预抓取点使用 OMPL 关节空间规划；下降、抬升、水平搬运、下降放置和撤离使用 `5 mm` 插值步长的笛卡尔直线路径。笛卡尔路径完成比例必须达到 `99.9%`，否则节点拒绝执行，避免执行截断路径或因重新选择 IK 分支而产生不必要的关节绕行。

这里的“抓住”同时包含可见的手指闭合和 MoveIt Planning Scene 的 attach/detach。它能正确显示夹爪贴合与物体跟随，但不计算接触力、摩擦、滑动或重力；这些属于后续 Gazebo/Isaac Sim 动力学阶段。

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
