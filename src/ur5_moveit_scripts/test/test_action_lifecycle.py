"""Test real-client action handling against a dummy server, never robot hardware."""

from threading import Thread
import time

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ur5_moveit_scripts.ur7e_pose_goal import PoseGoalNode


class ActionHarness(Node):
    """Use the production action logic against a test-specific ROS endpoint."""

    action = PoseGoalNode.action
    cancel_active = PoseGoalNode.cancel_active

    def __init__(self):
        """Create only an action client node, without motion or state interfaces."""
        super().__init__('action_test_client')
        self.active_handle = None
        self.result_future = None


@pytest.mark.parametrize('mode', ['success', 'failure', 'rejected', 'timeout'])
def test_action_lifecycle(mode):
    """Require accurate terminal status and acknowledged cancellation on timeout."""
    rclpy.init(domain_id=186)
    server_node = Node('dummy_execution_server')
    client = ActionHarness()
    canceled = []

    def execute(handle):
        result = ExecuteTrajectory.Result()
        if mode == 'timeout':
            deadline = time.monotonic() + 10.0
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                canceled.append(True)
                handle.canceled()
                result.error_code.val = MoveItErrorCodes.PREEMPTED
                return result
            handle.abort()
            result.error_code.val = MoveItErrorCodes.TIMED_OUT
        elif mode == 'failure':
            handle.abort()
            result.error_code.val = MoveItErrorCodes.CONTROL_FAILED
        else:
            handle.succeed()
            result.error_code.val = MoveItErrorCodes.SUCCESS
        return result

    server = ActionServer(
        server_node, ExecuteTrajectory, '/offline_test_execute', execute,
        callback_group=ReentrantCallbackGroup(),
        goal_callback=lambda request: (GoalResponse.REJECT if mode == 'rejected'
                                       else GoalResponse.ACCEPT),
        cancel_callback=lambda request: CancelResponse.ACCEPT,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if mode == 'success':
            result = client.action(ExecuteTrajectory, '/offline_test_execute',
                                   ExecuteTrajectory.Goal(), 2.0)
            assert result.error_code.val == MoveItErrorCodes.SUCCESS
        else:
            error = TimeoutError if mode == 'timeout' else RuntimeError
            with pytest.raises(error):
                client.action(ExecuteTrajectory, '/offline_test_execute',
                              ExecuteTrajectory.Goal(), 0.1 if mode == 'timeout' else 2.0)
        assert client.active_handle is None
        if mode == 'timeout':
            assert canceled == [True]
    finally:
        executor.shutdown()
        thread.join(timeout=5.0)
        server.destroy()
        server_node.destroy_node()
        client.destroy_node()
        rclpy.shutdown()
