"""Exercise real-hardware safety contracts without any robot connection."""

import ast
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from moveit_msgs.msg import AllowedCollisionEntry, MoveItErrorCodes, PlanningScene
from moveit_msgs.msg import RobotState, RobotTrajectory
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5_moveit_scripts import add_table_plane as table
from ur5_moveit_scripts.real_common import finite_vector, inspect_model, quaternion, scaling
from ur5_moveit_scripts.ur7e_pose_goal import (
    DEFAULTS, pose_goal, successful_result, validate_parameters, validate_trajectory,
)
import xacro
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    """Import the launch without executing any launch actions."""
    spec = importlib.util.spec_from_file_location(
        'real_launch', ROOT / 'launch/ur7e_real_moveit.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parameters():
    """Provide a numeric test target only; this is not a recommended hardware pose."""
    return dict(DEFAULTS, position=[0.1, 0.2, 0.3],
                quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])


@pytest.mark.parametrize('bad', [[], [1, 2], [1, 2, float('nan')],
                                 [1, 2, float('inf')], '123', [True, 2, 3]])
def test_bad_vectors(bad):
    """Reject malformed and nonfinite inputs."""
    with pytest.raises(ValueError):
        finite_vector(bad, 3, 'test')


@pytest.mark.parametrize('bad', [[0.0] * 4, [0.0, 0.0, 0.0, 2.0],
                                 [0.0, 0.0, 0.0, float('nan')]])
def test_bad_quaternions(bad):
    """Reject zero norm, nonunit and nonfinite orientation."""
    with pytest.raises(ValueError):
        quaternion(bad)


@pytest.mark.parametrize('bad', [0.0, -0.1, 2.0, float('nan'), float('inf')])
def test_bad_scaling(bad):
    """Prevent MoveIt's invalid-scaling fallback to full speed."""
    with pytest.raises(ValueError):
        scaling(bad)


def test_defaults_and_limits():
    """Require explicit goals and bounded settings."""
    assert scaling(1.0) == 0.1
    values = validate_parameters(parameters())
    assert values['velocity_scaling'] == values['acceleration_scaling'] == 0.05
    assert not values['execute']
    for key, value in [('position', []),
                       ('planning_attempts', 1.5), ('planning_timeout', float('nan')),
                       ('execution_timeout', -1.0), ('position', [200.0, 0.0, 0.0]),
                       ('execute', 'true'), ('max_joint_excursion', 4.0)]:
        with pytest.raises(ValueError):
            validate_parameters(dict(parameters(), **{key: value}))


def test_table_construction():
    """Build a genuine finite MoveIt collision box with a unit orientation."""
    obj = table.table_object(table.TABLE_DEFAULTS)
    assert obj.id == 'ur7e_work_surface'
    assert obj.header.frame_id == 'base_link'
    assert obj.primitives[0].type == obj.primitives[0].BOX
    assert list(obj.primitives[0].dimensions) == [4.0, 4.0, 0.01]
    assert obj.primitive_poses[0].orientation.w == 1.0
    assert obj.primitive_poses[0].position.z == -0.010
    assert obj.operation == obj.ADD
    for key, value in [('size_z', 0.0), ('size_x', float('nan')),
                       ('position_z', float('inf')), ('frame_id', 'tool0'), ('table_id', '')]:
        with pytest.raises(ValueError):
            table.table_object(dict(table.TABLE_DEFAULTS, **{key: value}))


def test_table_verified_not_just_published(monkeypatch):
    """Reject scene application failure, missing objects and changed geometry."""
    obj = table.table_object(table.TABLE_DEFAULTS)
    scene = PlanningScene()
    scene.world.collision_objects = [obj]
    monkeypatch.setattr(table, 'service_call', lambda *a, **kw: SimpleNamespace(success=True))
    monkeypatch.setattr(table, 'get_scene', lambda node: scene)
    table.apply_table(None, obj)
    monkeypatch.setattr(table, 'service_call', lambda *a, **kw: SimpleNamespace(success=False))
    with pytest.raises(RuntimeError, match='rejected'):
        table.apply_table(None, obj)
    monkeypatch.setattr(table, 'service_call', lambda *a, **kw: SimpleNamespace(success=True))
    changed = copy.deepcopy(obj)
    changed.primitive_poses[0].position.z = 0.5
    scene.world.collision_objects = [changed]
    with pytest.raises(RuntimeError, match='differs'):
        table.apply_table(None, obj)
    scene.world.collision_objects = []
    with pytest.raises(RuntimeError, match='missing'):
        table.require_table(scene, obj.id)


def test_table_collision_permissions():
    """Reject ACM entries that would allow any robot link through the table."""
    obj = table.table_object(table.TABLE_DEFAULTS)
    scene = PlanningScene()
    scene.world.collision_objects = [obj]
    scene.allowed_collision_matrix.entry_names = [obj.id, 'forearm_link']
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[False, True]),
        AllowedCollisionEntry(enabled=[True, False])]
    with pytest.raises(RuntimeError, match='allowed'):
        table.require_table(scene, obj.id)


def test_moveit_normalized_table_pose(monkeypatch):
    """Accept equivalent object/primitive offsets returned by actual MoveIt."""
    obj = table.table_object(table.TABLE_DEFAULTS)
    normalized = copy.deepcopy(obj)
    normalized.pose = copy.deepcopy(obj.primitive_poses[0])
    normalized.primitive_poses[0].position.z = 0.0
    scene = PlanningScene()
    scene.world.collision_objects = [normalized]
    monkeypatch.setattr(table, 'service_call', lambda *a, **kw: SimpleNamespace(success=True))
    monkeypatch.setattr(table, 'get_scene', lambda node: scene)
    table.apply_table(None, obj)


def test_plan_only_request():
    """Even execute=true must first send a strictly plan-only MoveGroup request."""
    values = validate_parameters(dict(parameters(), execute=True))
    goal = pose_goal(values, 'ur_manipulator', 'base_link', 'tool0', RobotState())
    assert goal.planning_options.plan_only
    assert not goal.planning_options.replan
    assert goal.request.pipeline_id == 'ompl'
    assert goal.request.max_velocity_scaling_factor == 0.05
    assert goal.request.num_planning_attempts == 5
    constraints = goal.request.goal_constraints[0]
    assert constraints.position_constraints[0].link_name == 'tool0'
    assert constraints.orientation_constraints[0].header.frame_id == 'base_link'


def trajectory():
    """Create a tiny synthetic planner result for validation tests only."""
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = [f'j{i}' for i in range(6)]
    for i in range(2):
        point = JointTrajectoryPoint(positions=[i * 0.01] * 6)
        point.time_from_start.sec = i
        result.joint_trajectory.points.append(point)
    return result


def test_trajectory_checks():
    """Reject jumps, bad timestamps, empty plans and mismatched current states."""
    current = {f'j{i}': 0.0 for i in range(6)}
    validate_trajectory(trajectory(), current, 0.35)
    bad = trajectory()
    bad.joint_trajectory.points[1].positions[2] = 1.0
    with pytest.raises(ValueError, match='excursion'):
        validate_trajectory(bad, current, 0.35)
    bad = trajectory()
    bad.joint_trajectory.points[0].positions[2] = 0.1
    with pytest.raises(ValueError, match='start'):
        validate_trajectory(bad, current, 0.35)
    bad = trajectory()
    bad.joint_trajectory.points[1].time_from_start.sec = 0
    with pytest.raises(ValueError, match='times'):
        validate_trajectory(bad, current, 0.35)
    with pytest.raises(ValueError):
        validate_trajectory(RobotTrajectory(), current, 0.35)


@pytest.mark.parametrize('status,code', [
    (GoalStatus.STATUS_ABORTED, 1),
    (GoalStatus.STATUS_SUCCEEDED, -4),
    (GoalStatus.STATUS_CANCELED, -7),
])
def test_failed_actions_never_report_success(status, code):
    """An action success status alone is insufficient for successful execution."""
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=MoveItErrorCodes(val=code)))
    with pytest.raises(RuntimeError, match='failed'):
        successful_result(result, 'test')


def test_official_model_contract():
    """Expand installed official UR7e descriptions locally, with no driver process."""
    urdf = xacro.process_file(
        get_package_share_directory('ur_robot_driver') + '/urdf/ur.urdf.xacro',
        mappings={'ur_type': 'ur7e', 'name': 'ur7e'}).toxml()
    srdf = xacro.process_file(
        get_package_share_directory('ur_moveit_config') + '/srdf/ur.srdf.xacro',
        mappings={'name': 'ur7e'}).toxml()
    group, base, tip, joints, real = inspect_model(urdf, srdf)
    assert (group, base, tip, len(joints), real) == (
        'ur_manipulator', 'base_link', 'tool0', 6, True)
    with pytest.raises(ValueError, match='Non-stock'):
        inspect_model(urdf.replace('</robot>', '<link name="gripper"/></robot>'), srdf)
    with pytest.raises(ValueError, match='ur7e'):
        inspect_model(urdf.replace('name="ur7e"', 'name="ur5"'), srdf)
    config = yaml.safe_load((Path(get_package_share_directory('ur_moveit_config'))
                             / 'config/moveit_controllers.yaml').read_text())
    controller = config['moveit_simple_controller_manager']['scaled_joint_trajectory_controller']
    assert controller['default'] and set(controller['joints']) == set(joints)


def test_real_launch_contains_no_motion_or_gripper(monkeypatch):
    """Inspect expanded launch actions without executing processes or using calibration data."""
    module = load_launch()
    assert module.generate_launch_description() is not None
    monkeypatch.setattr(module, 'validate_calibration', lambda path: '/external/calibration.yaml')
    context = LaunchContext()
    context.launch_configurations.update({
        'kinematics_params_file': '/external/calibration.yaml',
        'real_config_file': str(ROOT / 'config/real_motion_defaults.yaml'),
        'robot_ip': '192.168.1.100', 'launch_rviz': 'false',
    })
    actions = module.launch_setup(context)
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(nodes) == 1
    groups = [a for a in actions if isinstance(a, GroupAction)]
    driver = next(a for a in groups[0].get_sub_entities()
                  if isinstance(a, IncludeLaunchDescription))
    arguments = dict(driver.launch_arguments)
    assert arguments['ur_type'] == 'ur7e'
    assert arguments['use_mock_hardware'] == 'false'
    assert arguments['headless_mode'] == 'false'
    assert arguments['safety_limits'] == 'true'
    assert arguments['kinematics_params_file'] == '/external/calibration.yaml'
    assert arguments['description_file'].endswith('/ur_robot_driver/urdf/ur.urdf.xacro')
    source = (ROOT / 'launch/ur7e_real_moveit.launch.py').read_text()
    assert 'TimerAction' not in source and 'ur7e_pose_goal' not in source
    assert 'gripper' not in source
    with pytest.raises(ValueError, match='missing'):
        load_launch().validate_calibration('/does/not/exist.yaml')


def test_syntax_config_and_mock_guards():
    """Parse all package Python/YAML and preserve guarded legacy entry points."""
    for path in ROOT.rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    config = yaml.safe_load((ROOT / 'config/real_motion_defaults.yaml').read_text())
    defaults = config['ur7e_pose_goal']['ros__parameters']
    assert defaults == DEFAULTS
    assert 'position' not in defaults
    for path in ('ur5_pose_goal.py', 'ur5_joint_goal.py', 'ur5_go_home.py',
                 'ur5_gripper_state.py', 'motion_common.py'):
        assert 'require_ur5_mock(' in (ROOT / 'ur5_moveit_scripts' / path).read_text()
