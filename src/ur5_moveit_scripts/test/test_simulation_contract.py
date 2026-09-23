"""Test the simulation-only contract and parameter validation."""

from pathlib import Path

import pytest

from ur5_moveit_scripts.motion_common import finite_float_list


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'ur5_mock_moveit.launch.py'
GRIPPER_URDF = PACKAGE_ROOT / 'urdf' / 'ur5_parallel_gripper.urdf.xacro'
PICK_PLACE_FILE = (
    PACKAGE_ROOT / 'ur5_moveit_scripts' / 'ur5_pick_place_demo.py'
)
SETUP_FILE = PACKAGE_ROOT / 'setup.py'


def test_motion_vector_validation():
    """Accept finite vectors and reject invalid dimensions or values."""
    assert finite_float_list([1, 2, 3], 3, 'vector') == [1.0, 2.0, 3.0]

    with pytest.raises(ValueError, match='exactly 3'):
        finite_float_list([1, 2], 3, 'vector')

    with pytest.raises(ValueError, match='finite'):
        finite_float_list([1, float('inf'), 3], 3, 'vector')


def test_launch_hard_codes_mock_hardware():
    """Keep the project launch isolated from physical robot hardware."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'robot_ip': '127.0.0.1'" in source
    assert "'use_mock_hardware': 'true'" in source
    assert 'GroupAction(actions=[mock_control], scoped=True)' in source
    assert "'launch_dashboard_client': 'false'" in source
    assert "'use_tool_communication': 'false'" in source
    assert "'initial_joint_controller': 'scaled_joint_trajectory_controller'" in source
    assert "'activate_joint_controller': 'true'" in source
    assert "'description_file': PathJoinSubstitution(" in source
    assert "executable='ur5_gripper_state'" in source
    assert "LaunchConfiguration('robot_ip')" not in source
    assert "LaunchConfiguration('use_mock_hardware')" not in source


def test_pick_place_demo_contains_visible_scene_and_attachment_cycle():
    """Keep the table-top scene and simulated grasp/release cycle available."""
    source = PICK_PLACE_FILE.read_text(encoding='utf-8')
    setup_source = SETUP_FILE.read_text(encoding='utf-8')
    gripper_source = GRIPPER_URDF.read_text(encoding='utf-8')

    assert 'add_collision_box' in source
    assert 'add_collision_cylinder' in source
    assert 'attach_collision_object' in source
    assert 'detach_collision_object' in source
    assert "link_name='gripper_tcp'" in source
    assert 'GRIPPER_CLOSED_POSITION = 0.045' in source
    assert 'cartesian_max_step=0.005' in source
    assert 'cartesian_fraction_threshold=0.999' in source
    assert source.count('cartesian=True') == 5
    assert "controller_name: str = 'scaled_joint_trajectory_controller'" in source
    assert 'ur5_pick_place_demo = ' in setup_source
    assert 'ur5_gripper_state = ' in setup_source
    assert '<link name="gripper_tcp"/>' in gripper_source
    assert '<joint name="left_gripper_finger_joint" type="prismatic">' in gripper_source
    assert '<joint name="right_gripper_finger_joint" type="prismatic">' in gripper_source
    assert 'lower="0.045" upper="0.065"' in gripper_source
