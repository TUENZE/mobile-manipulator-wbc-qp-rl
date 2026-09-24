# Real UR7e: calibrated, tool-free MoveIt point-to-point planning

This is a ROS 2 **Jazzy / Ubuntu 24.04** workflow for one UR7e at
`192.168.1.100`, with **no end effector or gripper**. No WBC, QP, RL, Servo,
mobile base, grasping, or raw velocity control is involved.

The package remains named `ur5_moveit_scripts` to preserve existing commands.
Names beginning `ur5_` belong to mock mode. Use only `ur7e_real_moveit` and
`ur7e_pose_goal` for this real workflow. The table node adds geometry only.

## Architecture and guarantees

```text
Explicit target pose (tool0 in base_link)
    ↓
ur7e_pose_goal: fresh measured joint state, model/scene/input checks
    ↓
MoveIt 2 MoveGroup action (plan_only=true, pipeline=ompl)
    ↓
Planning Scene + configured table BOX + whole robot collision model
    ↓
Collision-aware OMPL path + time parameterization + solution validation
    ↓
DisplayTrajectory → inspect in RViz
    ↓ ONLY execute=true + terminal review + unchanged scene/start
MoveIt ExecuteTrajectory action
    ↓
scaled_joint_trajectory_controller/follow_joint_trajectory
    ↓
official ur_robot_driver → UR7e @ 192.168.1.100
```

The installed official SRDF defines group `ur_manipulator`, chain `base_link`
to `tool0`. The node discovers the group and joint names from the active
URDF/SRDF and checks the stock six-axis, tool-free UR7e contract. It rejects
other robots, extra links/joints, attached objects, and unknown hardware plugins.
Plan-only also supports a stock UR7e **mock** model for offline testing;
real execution requires the official UR hardware plugin.

The launch includes official `ur_robot_driver/ur_control.launch.py` and
`ur_moveit_config/ur_moveit.launch.py`. Jazzy MoveIt receives the **same calibrated
robot description** published by the driver's robot state publisher; it does not
generate a second nominal URDF. The official MoveIt controller configuration
selects `scaled_joint_trajectory_controller` by default, matching our driver
selection. No competing controller configuration is installed. The driver may
load other upstream controllers inactive; this project does not command them.

The launch starts no motion node, home motion, gripper state publisher, Servo,
or startup motion timer. It fixes real hardware, stock description, enabled
safety limits and `headless_mode=false`. Dashboard client and tool communication
are disabled. The table is applied through `/apply_planning_scene`, then geometry
is read back from `/get_planning_scene` and checked in a common frame. The node
exits after verification; MoveIt retains the object until removal/reset/restart.
Reapply it after restarting MoveIt.

## 1. Build and keep modes separate

```bash
cd ~/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Dependencies are declared in `package.xml`. This machine already has the
official Jazzy UR driver, UR description, MoveIt configuration and `pymoveit2`.
On a new machine, install the ROS Jazzy dependencies using your usual `rosdep`
workspace setup, including `ur_calibration` for calibration extraction.

In **every real-mode terminal**:

```bash
cd ~/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Use domain **42** for mock commands, **43** for real commands. These are DDS
discovery domains, not robot IP addresses. LOCALHOST discovery still allows the
driver to communicate with the robot over Ethernet. Do not start both modes in
one domain. **Stop any existing driver/MoveIt launch in its original terminals
before starting a new real driver**, even if it uses another ROS domain: both
would still connect to the same physical robot. The implementation does not
terminate existing sessions for you.

## 2. Ethernet, pendant and calibration

1. Connect Ethernet and confirm the existing network configuration:

   ```bash
   ping -c 4 192.168.1.100
   ```

2. Keep the workspace clear and have the pendant stop controls accessible.
   Configure the External Control URCap/program with this PC's robot-facing
   Ethernet address and script sender port `50002`, following the official UR
   setup. This project neither changes safety configuration nor powers or starts
   the robot program remotely.

3. The extracted file **`~/ur7e_calibration.yaml` was found locally**. The real
   launch defaults to this path and checks that it is readable, has finite
   calibration entries and contains an extraction hash. This is structural
   validation, **not proof that the file belongs to this robot**. Check the
   driver's calibration match at startup. No calibration values are committed.

   ```bash
   test -r "$HOME/ur7e_calibration.yaml"
   ```

   Only if extracting for the first time (do not overwrite a known-good file):

   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py \
     robot_ip:=192.168.1.100 \
     target_filename:="$HOME/ur7e_calibration.yaml"
   ```

   Unlike the upstream driver, this beginner-oriented launch **requires** a
   calibration file and does not silently fall back to nominal kinematics.
   Keep calibration outside the repository; `.gitignore` also excludes
   calibration YAML files. UR recommends robot-specific calibration because
   nominal Cartesian positions can differ by centimetres.
   [Official calibration documentation](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_robot_driver/ur_calibration/doc/index.html).

## 3. Table defaults (matching the original script)

The table is already configured as requested: the geometry from the original
`src/add_table_plane.py`, **4.0 × 4.0 × 0.01 m**, centre **[0, 0, -0.010]** in
`base_link`, with its top at **-0.005 m**. No measurement confirmation parameter
is required to plan. These are inherited, user-selected dimensions, not a claim
that the physical table was measured. Inspect alignment in RViz before execution.

For the commands below, copy the defaults to a local parameter file; no edits
are required to use the original table:

```bash
cp src/ur5_moveit_scripts/config/real_motion_defaults.yaml "$HOME/ur7e_real.yaml"
```

If adjustment is needed later, edit `add_table_plane.ros__parameters`:

| Parameter | Meaning |
|---|---|
| `frame_id` | `base_link` recommended; also `base` or `world`, never `tool0` |
| `table_id` | `ur7e_work_surface`; match the pose node's `table_id` |
| `size_x`, `size_y` | Measured XY extent, metres; cover the actual work surface |
| `size_z` | Physical thickness, metres, strictly positive |
| `position_x`, `position_y` | Box centre relative to the chosen frame |
| `position_z` | **Box centre**, computed as `top_surface_z - size_z / 2` |
The two sections are node-specific ROS parameters. Passing a params file to one
node does not configure another running node automatically.

The stock `world` and `base_link` origins coincide in this launch. `base` has a
180° Z rotation relative to `base_link`; XY signs can therefore differ. Do not
paste pendant Base coordinates directly as `base_link` coordinates. This V1
assumes the fixed robot mounting frame is correctly aligned with the horizontal
table. A tilted or externally positioned mounting requires a measured workcell
model; it is not handled by silently guessing a transform.

Do not put the box through the pedestal/base collision geometry, remove the table
to make a plan succeed, or allow collisions with it. If the real mounting geometry
needs a cutout, model the real cutout/workcell as additional finite geometry in a
future extension. Keep conservative clearance for measurement uncertainty. Add
other obstacles separately: this table-only V1 cannot perceive people or clutter.

## 4. Start the real stack (terminal 1)

After sourcing and setting domain 43 as above:

```bash
ros2 launch ur5_moveit_scripts ur7e_real_moveit.launch.py \
  robot_ip:=192.168.1.100 \
  kinematics_params_file:="$HOME/ur7e_calibration.yaml" \
  real_config_file:="$HOME/ur7e_real.yaml" \
  launch_rviz:=true
```

The IP and calibration path shown are already the defaults. `launch_rviz:=false`
is available, but inspect the trajectory using RViz on another terminal before
real execution. Wait for the table node's **`Table VERIFIED`** message. If it
fails, motion commands refuse a missing table; the driver may remain running.

With the driver ready, load your **External Control** program on the teach
pendant and press **Play** as required by your installation. Use a program with
External Control and no unrelated motion instructions. Starting that program
enables command reception; this launch sends no trajectory. `headless_mode=false`
is fixed deliberately. Headless mode would change the program-start workflow
and is not exposed by this V1.

Confirm in another real-mode terminal:

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base tool0
```

Check six arm joint states follow the physical robot and
`scaled_joint_trajectory_controller` is **active**. Compare `base → tool0` with
pendant TCP values using the pendant's Base feature and **zero tool TCP offset**.
The stock `tool0` frame is a nominal zero-tool TCP, not a gripper tip. Verify the
physical no-tool configuration on the pendant yourself; ROS does not alter it.

In RViz set Fixed Frame to `world` (or `base_link`), enable the MotionPlanning
display, select `ur_manipulator`, and enable Planning Scene Geometry. Inspect the
table and physical robot alignment. The planned-path topic is
`/display_planned_path`; enable the planned trajectory animation/loop as needed.

### Reapply or adjust the table (optional terminal 2)

The launch adds the table automatically. To reapply the configured table:

```bash
ros2 run ur5_moveit_scripts add_table_plane --ros-args \
  --params-file "$HOME/ur7e_real.yaml"
```

The equivalent direct command for the original table is:

```bash
ros2 run ur5_moveit_scripts add_table_plane --ros-args \
  -p frame_id:=base_link -p table_id:=ur7e_work_surface \
  -p size_x:=4.0 -p size_y:=4.0 -p size_z:=0.01 \
  -p position_x:=0.0 -p position_y:=0.0 -p position_z:=-0.010
```

Use decimal inputs (`2.0`, not `2`) for ROS double parameters. Save the same
values in your local YAML so restart uses the correct scene. Updating the
same ID replaces its geometry; it does not create another table. No Allowed
Collision Matrix exemptions are added. The motion node also rejects permissions
that would allow links to collide with the table.

## 5. Plan only (terminal 3)

Choose a small target near the **current measured** tool0 pose, with room for
the elbow and all other links. Start by reading the current pose in the actual
target frame:

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

Ctrl+C exits the transform viewer. Enter your deliberately chosen target in
metres and a unit quaternion in **x, y, z, w** order. There is no built-in home
or Cartesian goal:

```bash
read -r -p 'Target x y z in base_link (metres): ' X Y Z
read -r -p 'Target quaternion qx qy qz qw: ' QX QY QZ QW
ros2 run ur5_moveit_scripts ur7e_pose_goal --ros-args \
  --params-file "$HOME/ur7e_real.yaml" \
  -p "position:=[$X, $Y, $Z]" \
  -p "quaternion_xyzw:=[$QX, $QY, $QZ, $QW]" \
  -p execute:=false
```

Use decimal values for every vector element. The quaternion norm must be within
0.001 of 1; rounding error is normalized, arbitrary quaternions are rejected.
The position sanity bound of ±2 m is only an input/units check, **not** reachability
or table collision checking. MoveIt checks the whole robot and planned path.
TCP Z alone cannot tell whether the elbow intersects the table.

Wait for `Planning SUCCEEDED` and `PLAN ONLY complete`. Inspect the complete
trajectory in RViz, not just the final pose. OMPL plans in joint space; the
tool0 path is **not guaranteed to be a straight line**.

Defaults are 0.05 velocity and acceleration scaling; legal input is
`[0.001, 1.0]`, capped at **0.10** for this V1, with the effective value logged.
Invalid values are rejected rather than letting MoveIt substitute full speed.
The planning timeout is 10 s (0.1–60), attempts 5 (1–20), execution timeout 120 s
(1–600). An additional client response deadline bounds planning action waits.
`max_joint_excursion=0.35 rad` limits **every waypoint** relative to the current
six joint positions. A tiny Cartesian goal can still require a large joint
change near a singularity: inspect that failure, do not automatically raise the
limit. Valid configurable range is 0.001–π rad.

## 6. Explicit execution after review

In the same terminal (so target variables still exist):

```bash
ros2 run ur5_moveit_scripts ur7e_pose_goal --ros-args \
  --params-file "$HOME/ur7e_real.yaml" \
  -p "position:=[$X, $Y, $Z]" \
  -p "quaternion_xyzw:=[$QX, $QY, $QZ, $QW]" \
  -p execute:=true
```

**This command creates a new plan.** It displays that plan and waits up to 120 s
for you to type `EXECUTE` and press Enter in an interactive terminal. Inspect
this new trajectory before confirming: OMPL can return a different path on each
run. The reviewed in-memory trajectory is sent to ExecuteTrajectory without
replanning. Any other input, timeout, noninteractive stdin or Ctrl+C aborts.

Immediately before execution the node checks that the controller is active,
the scene/ACM has not changed, the table remains present with collision checking
enabled, and fresh joints remain within 0.01 rad of the original measured start.
If any check fails, start again with planning and review. It requires both a
successful action status and MoveIt SUCCESS error code to report execution
success. A timeout/interruption requests cancellation and waits briefly for
termination; it never reports that the physical robot is proven stopped.

RViz's own Plan & Execute button and external ROS clients do **not** pass through
these CLI guards. Use the documented CLI workflow for initial commissioning.
The launch enables the official controller so those clients can move the robot
when deliberately commanded; this is not an authorization/security boundary.

## 7. Stop and shut down

For a stop requiring immediate operator intervention, use the pendant Stop or
the appropriate physical emergency stop according to your lab procedure.
Ctrl+C in the pose node requests ROS action cancellation; communications or
process failure can prevent confirmation. Never rely on killing a terminal as
an emergency stop. After motion has stopped, stop the External Control program
on the pendant, then Ctrl+C the ROS launch terminal. A completed table node
needs no shutdown. Do not disable safety limits or clear protective stops
programmatically to make a plan run.

## Troubleshooting

| Symptom | Check / action |
|---|---|
| Cannot connect | `ping`, robot IP, PC subnet/interface and cable; confirm only one driver owns the robot. Do not alter safety settings. |
| External Control not running | Check URCap PC address/port, driver readiness, program loaded and Play pressed on pendant. Headless mode is intentionally disabled. |
| Controller inactive / execution rejected | Inspect `ros2 control list_controllers`, External Control and driver logs. The selected MoveIt controller is `scaled_joint_trajectory_controller`; do not activate competing motion controllers. |
| No `/joint_states` or fresh-state timeout | Same ROS domain in all terminals; joint state broadcaster/driver running; timestamps current, wall clocks correct; no simulated clock in this workflow. |
| Planning fails | Check MoveIt error code/log, current state validity, table frame, target units/orientation, group/model, joint limits and near-singular configurations. No execution follows failure. |
| Unreachable target | Read actual `base_link → tool0`, choose a nearby reachable target/orientation. IK can fail even when position alone seems reachable. |
| Table collision / invalid start | Compare whole arm and table in RViz. Correct measurements/frame or physically reconfigure using an approved operator procedure; never disable collision checking. |
| Safe TCP but elbow collision | Expected whole-robot collision rejection. Choose another pose/path with actual clearance. |
| Table in wrong frame | Remember `base` and `base_link` XY axes differ by 180°. Check TF and centre-vs-top convention. Reapply measured geometry and replan. |
| Missing/malformed calibration | Default is `~/ur7e_calibration.yaml`; supply an absolute valid path or extract using the command above. No nominal fallback. |
| Driver reports calibration mismatch | Stop Cartesian commissioning, confirm robot serial/file provenance, re-extract if needed. A YAML hash alone is not identity proof. |
| RViz differs from physical robot | Stop; verify UR7e model, calibration match, six joint states, no duplicate stack, mounting frame, and zero tool offset. Do not compensate by guessing target offsets. |
| Scene changed / robot moved during review | Plan again from fresh state and review again; the old plan is discarded. |
| Joint excursion rejected | Check for IK branch change, joint winding or singularity. Reduce/revise target; do not automatically increase the cap. |
| Execution timeout at low pendant speed | Driver scaling can prolong motion. Check cancellation/pendant state; deliberately increase timeout only if appropriate. No automatic retry. |
| Table absent after MoveIt restart | Reapply table and verify again. The object belongs to the running Planning Scene, not permanent storage. |
| Legacy UR5 command refuses to start | Correct behavior on UR7e. Use `ur7e_pose_goal`, or switch to domain 42 and launch the UR5 mock. |

The official UR configuration disables duration-based MoveIt trajectory
execution monitoring to accommodate scaled-controller slowdowns. This node
adds its own bounded wait and cancellation; pendant speed scaling remains
active. [Official UR MoveIt configuration notes](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_moveit_config/doc/index.html).

## Physical pre-flight and acceptance checklist

- [ ] Correct physical UR7e, securely mounted, no tool/gripper attached; workspace clear.
- [ ] Operator present, pendant stop accessible, lab safety procedure understood.
- [ ] Only one real driver; correct IP and all real terminals in domain 43.
- [ ] Correct robot-specific calibration file; driver reports no mismatch.
- [ ] External Control configured and running intentionally; controller active.
- [ ] `/joint_states` follows the robot; RViz posture and TF agree with the pendant.
- [ ] Tool-free/zero-offset `base → tool0` comparison checked; target frame is `base_link`.
- [ ] Original-script table is visible; its assumed frame/top height/extent are appropriate.
- [ ] `Table VERIFIED`, table visible in RViz, no allowed table collisions.
- [ ] Current whole robot is collision-free; other relevant obstacles accounted for.
- [ ] Small explicit nearby target; plan-only succeeds at 0.05/0.05 scaling.
- [ ] Entire path, including elbow/forearm/wrist, inspected; sensible joint excursion.
- [ ] **Plan-only negative test:** a deliberately colliding target/path is rejected;
      never execute that test and never move the physical table into the robot.
- [ ] `execute=true` newly generated plan inspected again before typing `EXECUTE`.
- [ ] One small physical motion tested locally; endpoint, smoothness, scaled speed and
      success/failure reporting verified by the operator.
- [ ] Pendant Stop and ROS cancellation behavior validated under the lab procedure.

## Limits and offline validation

No physical execution has been tested by the implementation agent. Offline
tests cannot establish calibration-to-serial matching, physical table alignment,
controller behavior on your robot, actual stopping distances, or actual path
clearance. MoveIt uses a static geometric model and discrete path collision
checking; it is not a safety-rated protective system. Scene changes during
physical execution and unmodelled obstacles remain the operator/workcell's
responsibility. No universal safe target is supplied.

The implementation uses native Jazzy MoveIt actions/services through `rclpy`.
Installed `pymoveit2` does support separate `plan()`/`execute()`; it remains in
the mock nodes. Native actions were chosen for this small new client to keep
bounded waits, cancellation, controller selection and both result codes explicit,
without refactoring the working mock planner.

For test commands, exact coverage and limitations, see
[ur7e_validation.md](ur7e_validation.md).
