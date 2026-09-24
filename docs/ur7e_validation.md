# UR7e implementation validation and change manifest

Environment inspected: Ubuntu 24.04, ROS 2 Jazzy, installed official UR driver,
UR description, UR MoveIt config, MoveIt messages/actions/services and pymoveit2.
Validation performed on 2026-09-23. No physical robot motion was commanded.
The user's existing driver/MoveIt processes were left running and untouched;
runtime test processes used dedicated domains and loopback mock hardware.

## Commands

```bash
cd ~/mobile_manipulator_wbc_qp_rl
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
git diff --check

# Argument inspection only: these do not start robot processes.
ros2 launch ur5_moveit_scripts ur7e_real_moveit.launch.py --show-args
ros2 launch ur5_moveit_scripts ur5_mock_moveit.launch.py --show-args

# Separate integration smoke test; never uses real hardware.
python3 src/ur5_moveit_scripts/test/offline_smoke.py
```

`offline_smoke.py` fixes ROS domain **187** and LOCALHOST discovery, refuses an
occupied test domain, and explicitly uses `use_mock_hardware=true` and
`robot_ip=127.0.0.1`. It expands the stock UR7e model with a test-only nonsingular
mock initial configuration in a temporary directory, starts the official mock
driver and MoveIt, derives its test target from that mock's forward kinematics,
and sends **plan-only** requests. These test initial joints/poses are never real
motion defaults. Test subprocesses are stopped in `finally`, including failures.
Logs are written under `/tmp/ur7e_offline_*`.

The action lifecycle unit tests use a dummy action server on domain **186** and
`/offline_test_execute`; that server does not contain a driver or robot controller.

## Results and coverage

- Baseline before changes: 5 passed, 1 skipped; no pre-existing test failures.
- Updated suite: **36 passed, 1 skipped; 0 errors, 0 failures**.
- Existing skipped test: copyright-header check, unchanged from baseline.
- Existing linter multiprocessing/fork warnings on Python 3.12 remain warnings.
- Build, Python syntax parsing/imports, ament flake8/pep257 and YAML loading pass.
- Both launch descriptions import and `--show-args` resolve successfully.
- Official UR7e model expands locally, including the user's external calibration;
  stock chain is `ur_manipulator`, `base_link → tool0`, six arm joints, no tool.
- Installed official MoveIt controller mapping matches
  `scaled_joint_trajectory_controller` and the discovered joint names.
- Validation tests reject malformed/nonfinite vectors, bad quaternions, invalid
  scaling, invalid timeouts/attempts, empty or oversized trajectories, bad starts,
  bad trajectory timestamps, and non-stock robot attachments.
- Scene tests cover finite box creation, ApplyPlanningScene rejection, missing
  objects, changed geometry, normalized equivalent object/primitive poses and
  table collision exemptions in the Allowed Collision Matrix.
- Launch tests check real hardware, pendant-controlled workflow, enabled safety
  limits, external calibration, stock description and absence of motion/gripper
  startup nodes. Existing mock launch isolation tests still pass.
- Dummy-server action tests verify successful completion, execution failure,
  rejected goals, and timeout with acknowledged cancellation. An action status
  alone cannot override a failing MoveIt error code.
- Offline integration checks the real pose client against stock UR7e mock
  MoveIt: table applied/read back, a nearby pose planned without execution,
  invalid table-colliding state rejected, legacy UR5 client rejected on UR7e.
- Original UR5 mock integration checks its guarded gripper/driver stack and
  `ur5_sim_demo` trajectory execution. Startup waits explicitly for joint states,
  the planning service and active trajectory controller. Its synchronous
  pymoveit2 calls use a single executor to avoid a competing spin thread.

MoveIt may print the upstream expected messages about loading the model from
`/robot_description` and having no Octomap 3D sensor plugin. Upstream launch
processes may require SIGTERM after a shutdown grace period. Neither is evidence
of physical execution validation.

## Relevant tree and every changed file

`+` is new; `~` is modified; entries without markers are preserved references.

```text
.gitignore ~
README.md ~
docs/
  ur5_mock_simulation_v1.md ~
  ur5_moveit_computation_graph.md ~
  ur7e_real_hardware.md +
  ur7e_validation.md +
src/
  add_table_plane.py                  # original geometry reference, preserved
  ur5_moveit_scripts/
    package.xml ~
    setup.py ~
    launch/
      ur5_mock_moveit.launch.py       # preserved mock-only launch
      ur7e_real_moveit.launch.py +
    config/
      motion_defaults.yaml           # mock defaults preserved
      real_motion_defaults.yaml +
    urdf/
      ur5_parallel_gripper.urdf.xacro # mock geometry preserved
    ur5_moveit_scripts/
      real_common.py +
      add_table_plane.py +
      ur7e_pose_goal.py +
      motion_common.py ~
      ur5_go_home.py ~
      ur5_gripper_state.py ~
      ur5_joint_goal.py ~
      ur5_pose_goal.py ~
      ur5_sim_demo.py ~
      ur5_pick_place_demo.py          # preserved; guarded through motion_common
    test/
      test_real_contract.py +
      test_action_lifecycle.py +
      offline_smoke.py +
      test_simulation_contract.py    # existing tests preserved
```

| File(s) | Purpose of change |
|---|---|
| `.gitignore` | Keep machine-specific calibration YAML outside version control. |
| `README.md` | Explain both modes and link the complete operating/validation guides. |
| `docs/ur5_mock_simulation_v1.md` | Preserve mock instructions, clarify real entry point and domain separation. |
| `docs/ur5_moveit_computation_graph.md` | Mark historical UR5 examples as mock-only and direct real users to the new guide. |
| `docs/ur7e_real_hardware.md` | Exact commands, calibration, frames, original-script table, explicit review, troubleshooting and physical checklist. |
| `docs/ur7e_validation.md` | Validation evidence, limitations and this file manifest. |
| `package.xml` | Declare direct Jazzy message, TF, YAML and description dependencies; describe both modes. |
| `setup.py` | Register the two new executables and update package description; retain all old commands. |
| `launch/ur7e_real_moveit.launch.py` | Separate official stock calibrated UR7e stack; no startup motion or gripper. |
| `config/real_motion_defaults.yaml` | Safe real motion settings, no target, original table geometry (4 × 4 × 0.01 m at z=-0.010). |
| `real_common.py` | Input/model validation, bounded ROS waits and mock-only guard. |
| `add_table_plane.py` (packaged) | Configurable finite collision box, service acknowledgement and geometric read-back verification. |
| `ur7e_pose_goal.py` | Fresh-state pose planning, trajectory review, explicit execution and accurate result/cancellation handling. |
| `motion_common.py` | Guard shared UR5 mock motion interface creation. |
| `ur5_go_home.py` | Reject non-UR5/mock model before constructing a motion interface. |
| `ur5_joint_goal.py` | Same model/hardware guard for the legacy joint command. |
| `ur5_pose_goal.py` | Same model/hardware guard for the legacy pose command. |
| `ur5_gripper_state.py` | Refuse to publish mock finger states against a real model. |
| `ur5_sim_demo.py` | Retain mock trajectory; remove competing executor and wait for readiness explicitly. |
| `test_real_contract.py` | Validation, geometry, scene, model, controller, launch, config and isolation tests. |
| `test_action_lifecycle.py` | Real client action protocol tests with a dummy server, including cancellation. |
| `test/offline_smoke.py` | Reproducible loopback-only UR7e planning and UR5 mock execution integration checks. |

The original standalone `src/add_table_plane.py` remains as the geometry
reference. For the real workflow use the **packaged** `ros2 run
ur5_moveit_scripts add_table_plane`, which verifies scene acceptance.
No calibration values, fake calibration files, or changes to Git history are
included. No commit, push or merge was performed.

## Still requiring local physical validation

Use the [pre-flight checklist](ur7e_real_hardware.md#physical-pre-flight-and-acceptance-checklist).
The robot's calibration match, real TCP/frame alignment, suitability of the
inherited table geometry, actual low-speed execution, stopping behavior and
RViz/physical agreement must be checked by the operator. RViz visual inspection
was not performed by the headless integration tests. Tests of a dummy action
server do not demonstrate execution on UR7e hardware.
