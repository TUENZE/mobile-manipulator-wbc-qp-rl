"""Validation and bounded ROS calls for the stock, tool-free UR7e workflow."""

import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def finite_vector(values, length, name):
    """Reject malformed, nonnumeric or nonfinite vectors."""
    if isinstance(values, (str, bytes)) or len(values) != length:
        raise ValueError(f'{name} requires exactly {length} numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise ValueError(f'{name} requires numbers')
    result = [float(v) for v in values]
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must be finite')
    return result


def bounded(value, low, high, name):
    """Validate a finite scalar in an inclusive range."""
    result = finite_vector([value], 1, name)[0]
    if not low <= result <= high:
        raise ValueError(f'{name} must be in [{low}, {high}]')
    return result


def scaling(value):
    """Reject invalid scaling; cap valid requests at 10 percent for this V1."""
    return min(bounded(value, 0.001, 1.0, 'scaling'), 0.10)


def quaternion(values):
    """Require a unit xyzw quaternion, allowing only rounding error."""
    result = finite_vector(values, 4, 'quaternion_xyzw')
    norm = math.sqrt(sum(v * v for v in result))
    if abs(norm - 1.0) > 0.001:
        raise ValueError('quaternion_xyzw must have unit norm (within 0.001)')
    return [v / norm for v in result]


def wait_until(node, predicate, timeout, label):
    """Spin with a monotonic deadline, including on systems without ROS time."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(f'Timed out waiting for {label}')
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
    if not rclpy.ok():
        raise RuntimeError(f'ROS shut down waiting for {label}')


def future_result(node, future, timeout, label):
    """Wait for a ROS response and propagate errors instead of assuming success."""
    wait_until(node, future.done, timeout, label)
    result = future.result()
    if result is None:
        raise RuntimeError(f'Empty response from {label}')
    return result


def service_call(node, srv_type, name, request, timeout=10.0):
    """Call one service with bounded discovery and response waits."""
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'Service unavailable: {name}')
        return future_result(node, client.call_async(request), timeout, name)
    finally:
        node.destroy_client(client)


def read_description(node, timeout=15.0):
    """Read the driver-published model, refusing ambiguous publishers."""
    messages = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(String, '/robot_description', messages.append, qos)
    try:
        wait_until(node, lambda: bool(messages), timeout, '/robot_description')
        if node.count_publishers('/robot_description') != 1:
            raise RuntimeError('Expected exactly one robot description publisher')
        return messages[-1].data
    finally:
        node.destroy_subscription(sub)


def require_ur5_mock(node):
    """Prevent legacy demo clients from sending goals to a real MoveIt stack."""
    model = ET.fromstring(read_description(node))
    plugins = [p.text for p in model.findall('ros2_control/hardware/plugin')]
    if model.get('name') != 'ur5' or plugins != ['mock_components/GenericSystem']:
        raise RuntimeError('This UR5 demo requires the UR5 mock launch; refusing motion')


def inspect_model(description, semantic):
    """Discover the official arm chain and reject attachments or other robots."""
    model = ET.fromstring(description)
    srdf = ET.fromstring(semantic)
    if model.get('name') != 'ur7e' or srdf.get('name') != 'ur7e':
        raise ValueError('Both active URDF and SRDF must describe ur7e')
    groups = [g for g in srdf.findall('group') if g.find('chain') is not None]
    if len(groups) != 1:
        raise ValueError('Expected the official single UR arm chain')
    group = groups[0]
    chain = group.find('chain')
    base, tip = chain.get('base_link'), chain.get('tip_link')
    if (base, tip) != ('base_link', 'tool0'):
        raise ValueError('Expected the stock base_link to tool0 chain')
    joints = {j.find('child').get('link'): j for j in model.findall('joint')}
    names = []
    link = tip
    visited = set()
    while link != base:
        if link in visited:
            raise ValueError('Cyclic robot chain')
        visited.add(link)
        joint = joints[link]
        if joint.get('type') != 'fixed':
            names.insert(0, joint.get('name'))
        link = joint.find('parent').get('link')
        if len(names) > 6:
            raise ValueError('Not a stock six-axis arm')
    active = [j.get('name') for j in model.findall('joint')
              if j.get('type') != 'fixed']
    # URDF links inspected in the installed official UR7e description.
    stock_links = {
        'world', 'base_link', 'base_link_inertia', 'shoulder_link',
        'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link',
        'wrist_3_link', 'flange', 'tool0', 'base', 'ft_frame',
    }
    if len(names) != 6 or set(active) != set(names):
        raise ValueError('Extra or missing active joints; no tool attachments allowed')
    if not {x.get('name') for x in model.findall('link')} <= stock_links:
        raise ValueError('Non-stock links detected; no tool attachments allowed')
    plugins = [p.text for p in model.findall('ros2_control/hardware/plugin')]
    real = plugins == ['ur_robot_driver/URPositionHardwareInterface']
    if not real and plugins != ['mock_components/GenericSystem']:
        raise ValueError('Unknown hardware plugin')
    return group.get('name'), base, tip, names, real
