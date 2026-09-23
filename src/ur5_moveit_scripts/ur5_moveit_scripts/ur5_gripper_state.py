"""Publish kinematic state for the simulation-only parallel gripper."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


OPEN_POSITION = 0.065
CLOSED_POSITION = 0.045
MAX_SPEED = 0.04
JOINT_NAMES = [
    'left_gripper_finger_joint',
    'right_gripper_finger_joint',
]


class GripperStateNode(Node):
    """Interpolate commanded opening and publish both finger joint states."""

    def __init__(self) -> None:
        """Create publishers, subscriber and the 20 Hz state timer."""
        super().__init__('ur5_gripper_state')
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )
        self._command_subscription = self.create_subscription(
            Float64,
            '/gripper/command',
            self._command_callback,
            command_qos,
        )
        self._position = OPEN_POSITION
        self._target = OPEN_POSITION
        self._last_update = time.monotonic()
        self._timer = self.create_timer(0.05, self._publish_state)
        self.get_logger().info('Parallel gripper ready in the open position.')

    def _command_callback(self, message: Float64) -> None:
        self._target = min(OPEN_POSITION, max(CLOSED_POSITION, message.data))
        self.get_logger().info(f'Gripper target opening: {self._target:.3f} m')

    def _publish_state(self) -> None:
        now = time.monotonic()
        elapsed = min(now - self._last_update, 0.2)
        self._last_update = now
        maximum_step = MAX_SPEED * elapsed
        error = self._target - self._position
        if abs(error) <= maximum_step:
            self._position = self._target
        else:
            self._position += maximum_step if error > 0.0 else -maximum_step

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [self._position, self._position]
        message.velocity = [0.0, 0.0]
        self._joint_state_publisher.publish(message)


def main(args=None):
    """Run the gripper state publisher until shutdown."""
    rclpy.init(args=args)
    node = GripperStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
