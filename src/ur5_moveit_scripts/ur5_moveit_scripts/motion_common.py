"""Shared, version-pinned setup for the UR5 MoveIt example nodes."""

from dataclasses import dataclass
from threading import Thread
from typing import Sequence

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as ur_robot

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


DEFAULT_PLANNER = ''
DEFAULT_SCALING = 0.10


def finite_float_list(values: Sequence[float], length: int, name: str) -> list[float]:
    """Return validated finite floats with an exact expected length."""
    import math

    result = [float(value) for value in values]
    if len(result) != length:
        raise ValueError(
            f'{name} must contain exactly {length} values; got {len(result)}'
        )
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} must contain only finite values')
    return result


def declare_motion_parameters(node: Node) -> None:
    """Declare parameters shared by every motion command."""
    node.declare_parameter('planner_id', DEFAULT_PLANNER)
    node.declare_parameter('max_velocity', DEFAULT_SCALING)
    node.declare_parameter('max_acceleration', DEFAULT_SCALING)


def _scaling_parameter(node: Node, name: str) -> float:
    value = float(node.get_parameter(name).value)
    if not 0.0 < value <= 1.0:
        raise ValueError(
            f'{name} must be in the interval (0.0, 1.0]; got {value}'
        )
    return value


def create_moveit_interface(node: Node) -> MoveIt2:
    """Create the official UR MoveIt interface using the default tool0 TCP."""
    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur_robot.joint_names(),
        base_link_name=ur_robot.base_link_name(),
        end_effector_name=ur_robot.end_effector_name(),
        group_name=ur_robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = str(node.get_parameter('planner_id').value)
    moveit2.max_velocity = _scaling_parameter(node, 'max_velocity')
    moveit2.max_acceleration = _scaling_parameter(node, 'max_acceleration')
    return moveit2


@dataclass
class BackgroundExecutor:
    """Own the executor thread used for MoveIt actions and state feedback."""

    executor: MultiThreadedExecutor
    thread: Thread

    @classmethod
    def start(cls, node: Node) -> 'BackgroundExecutor':
        """Start a two-thread executor for a node."""
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        return cls(executor=executor, thread=thread)

    def stop(self) -> None:
        """Stop callbacks and join the background thread."""
        self.executor.shutdown()
        self.thread.join(timeout=2.0)


def execute_and_report(node: Node, moveit2: MoveIt2, description: str) -> None:
    """Wait for an already-requested motion and raise on failure."""
    if not moveit2.wait_until_executed():
        error_code = moveit2.get_last_execution_error_code()
        raise RuntimeError(f'{description} failed; MoveIt error: {error_code}')
    node.get_logger().info(f'{description} completed successfully.')


def shutdown(node: Node, background: BackgroundExecutor) -> None:
    """Release ROS resources in a consistent order."""
    background.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
