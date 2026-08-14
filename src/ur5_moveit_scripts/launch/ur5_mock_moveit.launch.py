"""Start a local-only UR5 MoveIt stack backed by mock ros2_control hardware."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Build a simulation-only launch description.

    The hardware mode and loopback address are deliberately not launch
    arguments. This package therefore cannot be pointed at a physical robot by
    accidentally changing a command-line option.
    """
    launch_rviz = LaunchConfiguration('launch_rviz')
    run_demo = LaunchConfiguration('run_demo')

    mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py',
                ]
            )
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': '127.0.0.1',
            'use_mock_hardware': 'true',
            'mock_sensor_commands': 'false',
            'headless_mode': 'true',
            'launch_dashboard_client': 'false',
            'use_tool_communication': 'false',
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py',
                ]
            )
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'launch_rviz': launch_rviz,
            # Mock hardware mirrors commands directly and does not publish a
            # Gazebo /clock topic, so ROS wall time is the correct clock.
            'use_sim_time': 'false',
            'launch_servo': 'false',
        }.items(),
    )

    demo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ur5_moveit_scripts',
                executable='ur5_sim_demo',
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare('ur5_moveit_scripts'),
                            'config',
                            'motion_defaults.yaml',
                        ]
                    )
                ],
            )
        ],
        condition=IfCondition(run_demo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with the official UR MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'run_demo',
                default_value='false',
                choices=['true', 'false'],
                description='Run one conservative mock joint trajectory after startup.',
            ),
            GroupAction(actions=[mock_control], scoped=True),
            moveit,
            demo,
        ]
    )
