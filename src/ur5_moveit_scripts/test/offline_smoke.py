"""Run integration checks on loopback mock hardware in a dedicated ROS domain."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ur5_moveit_scripts.add_table_plane import apply_table, TABLE_DEFAULTS, table_object
from ur5_moveit_scripts.real_common import service_call, wait_until
import xacro
import yaml


def command(args, expected=0):
    """Run a bounded local CLI command, preserving its output on failure."""
    result = subprocess.run(args, text=True, capture_output=True, timeout=90)
    print(result.stdout, result.stderr, flush=True)
    assert result.returncode == expected, (args, result.returncode)
    if expected == 0:
        assert 'Traceback' not in result.stderr and 'Exception in thread' not in result.stderr
    return result.stdout + result.stderr


def main():
    """Check real-node planning/collisions on UR7e mock, then legacy UR5 mock motion."""
    # Hard-coded test-only domain/loopback; never inherit a live robot domain.
    os.environ['ROS_DOMAIN_ID'] = '187'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    directory = Path(tempfile.mkdtemp(prefix='ur7e_offline_'))
    print(f'OFFLINE MOCK TEST: domain 187, logs {directory}', flush=True)
    processes = []
    streams = []
    node = None

    def launch(label, args):
        stream = (directory / f'{label}.log').open('w')
        streams.append(stream)
        process = subprocess.Popen(['ros2', 'launch'] + args, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)

    def stop():
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        processes.clear()

    try:
        rclpy.init()
        node = Node('offline_smoke')
        # Refuse an already populated test domain rather than disturbing another run.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.count_publishers('/robot_description') == 0, 'Test domain is occupied'
        # Test-only nonsingular initial state. Never used by the real launch/client.
        initial = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.2,
                   'elbow_joint': 0.8, 'wrist_1_joint': -1.2,
                   'wrist_2_joint': -1.3, 'wrist_3_joint': 0.0}
        initial_path = directory / 'mock_initial.yaml'
        initial_path.write_text(yaml.safe_dump(initial))
        description = xacro.process_file(
            get_package_share_directory('ur_robot_driver') + '/urdf/ur.urdf.xacro',
            mappings={'name': 'ur7e', 'ur_type': 'ur7e', 'robot_ip': '127.0.0.1',
                      'use_mock_hardware': 'true', 'headless_mode': 'true',
                      'safety_limits': 'true', 'initial_positions_file': str(initial_path)})
        model_path = directory / 'ur7e_mock.urdf'
        model_path.write_text(description.toxml())
        launch('ur7e_driver_mock', [
            'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur7e',
            'robot_ip:=127.0.0.1', 'use_mock_hardware:=true',
            'launch_dashboard_client:=false', 'headless_mode:=true', 'launch_rviz:=false',
            f'description_file:={model_path}'])
        launch('ur7e_moveit', [
            'ur_moveit_config', 'ur_moveit.launch.py',
            'ur_type:=ur7e', 'launch_rviz:=false', 'launch_servo:=false'])
        samples = []
        node.create_subscription(JointState, '/joint_states', samples.append,
                                 qos_profile_sensor_data)
        wait_until(node, lambda: bool(samples), 45.0, 'mock states')
        command(['ros2', 'run', 'ur5_moveit_scripts', 'add_table_plane'])
        state = RobotState(joint_state=samples[-1])
        validity = service_call(node, GetStateValidity, '/check_state_validity',
                                GetStateValidity.Request(robot_state=state), timeout=30.0)
        assert validity.valid, 'Stock mock start intersects placeholder table'
        request = GetPositionFK.Request(robot_state=state, fk_link_names=['tool0'])
        request.header.frame_id = 'base_link'
        fk = service_call(node, GetPositionFK, '/compute_fk', request)
        assert fk.error_code.val == 1
        pose = fk.pose_stamped[0].pose
        p, q = pose.position, pose.orientation
        target = [p.x, p.y, p.z - 0.01]  # derived from THIS mock state, never a real default
        args = ['ros2', 'run', 'ur5_moveit_scripts', 'ur7e_pose_goal', '--ros-args',
                '-p', f'position:={target}', '-p',
                f'quaternion_xyzw:={[q.x, q.y, q.z, q.w]}',
                '-p', 'execute:=false']
        output = command(args)
        assert 'PLAN ONLY complete' in output
        # Running a legacy demo against UR7e must fail before requesting motion.
        command(['ros2', 'run', 'ur5_moveit_scripts', 'ur5_sim_demo'], expected=1)
        blocked = dict(TABLE_DEFAULTS, size_x=4.0, size_y=4.0, size_z=4.0, position_z=0.5)
        apply_table(node, table_object(blocked))
        validity = service_call(node, GetStateValidity, '/check_state_validity',
                                GetStateValidity.Request(robot_state=state))
        assert not validity.valid
        output = command(args, expected=1)
        assert 'Current state is invalid/in collision' in output
        print('PASS: UR7e mock planning, table collision rejection, legacy guard.', flush=True)
        stop()
        launch('ur5_mock', ['ur5_moveit_scripts', 'ur5_mock_moveit.launch.py',
                            'launch_rviz:=false', 'run_demo:=false'])
        # CLI waits for its guarded model and controller action; no real endpoints exist.
        command(['ros2', 'run', 'ur5_moveit_scripts', 'ur5_sim_demo'])
        print('PASS: original UR5 mock trajectory execution.', flush=True)
    finally:
        stop()
        for stream in streams:
            stream.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
