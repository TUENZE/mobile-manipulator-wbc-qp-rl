"""Connect the calibrated stock UR7e and MoveIt; never launch a motion client."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur5_moveit_scripts.real_common import finite_vector
import yaml


def validate_calibration(filename):
    """Require a real extraction file; never fall back to nominal kinematics."""
    path = Path(filename).expanduser()
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f'Calibration file missing: {path}. See docs/ur7e_real_hardware.md')
    data = yaml.safe_load(path.read_text())['kinematics']
    if not str(data.get('hash', '')).startswith('calib_'):
        raise ValueError('Expected a robot-extracted calibration hash')
    for link in ('shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3'):
        finite_vector([data[link][key] for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')],
                      6, f'calibration {link}')
    return str(path)


def launch_setup(context):
    """Validate local inputs before starting any hardware process."""
    calibration = validate_calibration(
        LaunchConfiguration('kinematics_params_file').perform(context))
    config = LaunchConfiguration('real_config_file').perform(context)
    if not Path(config).is_file():
        raise ValueError(f'Real configuration file missing: {config}')
    driver_share = Path(get_package_share_directory('ur_robot_driver'))
    moveit_share = Path(get_package_share_directory('ur_moveit_config'))
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver_share / 'launch/ur_control.launch.py')),
        launch_arguments={
            'ur_type': 'ur7e',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': 'false',
            'headless_mode': 'false',
            'launch_dashboard_client': 'false',
            'use_tool_communication': 'false',
            'tool_voltage': '0',
            'safety_limits': 'true',
            'tf_prefix': '',
            'launch_rviz': 'false',
            'description_launchfile': str(driver_share / 'launch/ur_rsp.launch.py'),
            'description_file': str(driver_share / 'urdf/ur.urdf.xacro'),
            'kinematics_params_file': calibration,
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / 'launch/ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': 'ur7e',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_servo': 'false',
            'use_sim_time': 'false',
        }.items(),
    )
    return [
        LogInfo(msg=f'REAL UR7e: calibration={calibration}. No motion is requested. '
                'Inspect the configured table and scene before any execution.'),
        GroupAction(actions=[driver], scoped=True),
        GroupAction(actions=[moveit], scoped=True),
        Node(package='ur5_moveit_scripts', executable='add_table_plane',
             parameters=[config], output='screen'),
    ]


def generate_launch_description():
    """Expose only the real robot's connection, calibration, scene and RViz inputs."""
    share = Path(get_package_share_directory('ur5_moveit_scripts'))
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('kinematics_params_file',
                              default_value=str(Path.home() / 'ur7e_calibration.yaml'),
                              description='Absolute path to this robot calibration YAML.'),
        DeclareLaunchArgument('real_config_file',
                              default_value=str(share / 'config/real_motion_defaults.yaml'),
                              description='ROS parameters including table geometry.'),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
