# Mobile Manipulator WBC QP RL

FURP research project repository for tracking weekly progress, meeting notes, source code, and final showcase material.

## Project Info

| Field | Entry |
|---|---|
| Student name(s) | Tu Enze |
| Project title | Mobile Manipulator WBC QP RL |
| Project tag | MobileManipulatorWBCQPRL |
| Track | Research |
| Supervising faculty | Chiew-Foong Kwong|
| Project lead | Fuhua Jia |
| Team or individual | Individual |
| Cited paper being replicated | TBD |

**One-line summary:** This project studies whole-body-control and QP/RL-based methods for mobile manipulator control.

## Simulation V1

The first local-only UR5 simulation baseline is documented in
[`docs/ur5_mock_simulation_v1.md`](docs/ur5_mock_simulation_v1.md). It uses
MoveIt 2 with mock ros2_control hardware and the stock `tool0` end-effector
frame fitted with a kinematic parallel gripper; it does not connect to a
physical robot.

## Two separate MoveIt modes (ROS 2 Jazzy)

| Mode | Launch | Model / hardware |
|---|---|---|
| Local mock | `ur5_mock_moveit.launch.py` | UR5, mock gripper, loopback-only mock hardware |
| **Real robot** | `ur7e_real_moveit.launch.py` | Calibrated stock UR7e, **no tool**, `192.168.1.100` |

Start with the [real UR7e operating guide](docs/ur7e_real_hardware.md) for the
complete build, calibration, table setup, plan-only and explicit execution
sequence, troubleshooting, and physical validation checklist. The default
calibration path is `~/ur7e_calibration.yaml`; calibration data stays outside Git.
No real trajectory is requested by launch. The CLI defaults to plan-only and
requires an explicit pose. Execution uses 5% velocity/acceleration scaling and
requires review of the newly planned trajectory in the terminal.

Keep mock and real sessions in different ROS domains, and never run two drivers
for the same physical robot. The existing package name `ur5_moveit_scripts` is
retained for backwards compatibility. The current implementation is MoveIt
point-to-point planning; WBC/QP/RL, Servo and real grasping are not implemented.

Build and start the existing mock:

```bash
cd ~/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ur5_moveit_scripts ur5_mock_moveit.launch.py
```

All mock demo clients now verify the active model is UR5 **mock hardware** before
creating a motion interface. Do not use the old UR5 goal/home/pick-place commands
for a physical robot. See [validation results](docs/ur7e_validation.md).

## Repository Link

Public GitHub repository: https://github.com/TUENZE/mobile-manipulator-wbc-qp-rl
