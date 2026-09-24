"""Plan a stock UR7e pose with MoveIt and explicitly review before execution."""

import copy
import math
import select
import sys
import time

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint, PositionConstraint, RobotState
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from ur5_moveit_scripts.add_table_plane import get_scene, require_table
from ur5_moveit_scripts.real_common import (
    bounded, finite_vector, future_result, inspect_model, quaternion,
    read_description, scaling, service_call, wait_until,
)


CONTROLLER = 'scaled_joint_trajectory_controller'
DEFAULTS = {
    'execute': False, 'velocity_scaling': 0.05, 'acceleration_scaling': 0.05,
    'planning_timeout': 10.0, 'planning_attempts': 5, 'execution_timeout': 120.0,
    'max_joint_excursion': 0.35, 'table_id': 'ur7e_work_surface',
}


def validate_parameters(values):
    """Reject unsafe or ambiguous inputs before contacting any motion endpoint."""
    result = dict(values)
    result['position'] = finite_vector(values['position'], 3, 'position')
    if any(abs(v) > 2.0 for v in result['position']):
        raise ValueError('Position outside V1 sanity bounds (+/-2 m); check units')
    result['quaternion_xyzw'] = quaternion(values['quaternion_xyzw'])
    for key in ('velocity_scaling', 'acceleration_scaling'):
        result[key] = scaling(values[key])
    for key, low, high in (
        ('planning_timeout', 0.1, 60.0), ('execution_timeout', 1.0, 600.0),
        ('max_joint_excursion', 0.001, math.pi),
    ):
        result[key] = bounded(values[key], low, high, key)
    attempts = values['planning_attempts']
    if type(attempts) is not int or not 1 <= attempts <= 20:
        raise ValueError('planning_attempts must be an integer in [1, 20]')
    for key in ('execute',):
        if type(values[key]) is not bool:
            raise ValueError(f'{key} must be boolean')
    if not isinstance(values['table_id'], str) or not values['table_id'].strip():
        raise ValueError('table_id must be nonempty')
    return result


def pose_goal(values, group, base, tip, state):
    """Build a plan-only MoveGroup goal with explicit current state and constraints."""
    goal = MoveGroup.Goal()
    goal.planning_options.plan_only = True
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    request = goal.request
    request.group_name = group
    request.pipeline_id = 'ompl'
    request.num_planning_attempts = values['planning_attempts']
    request.allowed_planning_time = values['planning_timeout']
    request.max_velocity_scaling_factor = values['velocity_scaling']
    request.max_acceleration_scaling_factor = values['acceleration_scaling']
    request.start_state = state
    position = PositionConstraint()
    position.header.frame_id = base
    position.link_name = tip
    position.weight = 1.0
    region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values['position']
    pose.orientation.w = 1.0
    position.constraint_region.primitives = [region]
    position.constraint_region.primitive_poses = [pose]
    orientation = OrientationConstraint()
    orientation.header.frame_id = base
    orientation.link_name = tip
    q = values['quaternion_xyzw']
    orientation.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    orientation.absolute_x_axis_tolerance = 0.01
    orientation.absolute_y_axis_tolerance = 0.01
    orientation.absolute_z_axis_tolerance = 0.01
    orientation.weight = 1.0
    request.goal_constraints = [Constraints(
        position_constraints=[position], orientation_constraints=[orientation])]
    return goal


def validate_trajectory(trajectory, current, excursion):
    """Reject empty, malformed, discontinuous or unexpectedly large motions."""
    jt = trajectory.joint_trajectory
    if (len(jt.joint_names) != len(current) or set(jt.joint_names) != set(current)
            or len(jt.points) < 2 or trajectory.multi_dof_joint_trajectory.points):
        raise ValueError('Trajectory must contain only the six arm joints and >=2 points')
    start = [current[name] for name in jt.joint_names]
    previous_time = -1.0
    for index, point in enumerate(jt.points):
        positions = finite_vector(point.positions, len(start), 'trajectory positions')
        for field in ('velocities', 'accelerations'):
            vector = getattr(point, field)
            if vector:
                finite_vector(vector, len(start), field)
        timestamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if timestamp < 0 or timestamp <= previous_time:
            raise ValueError('Trajectory times must be nonnegative and increasing')
        previous_time = timestamp
        if index == 0 and any(abs(a - b) > 0.01 for a, b in zip(start, positions)):
            raise ValueError('Trajectory start differs from measured current state')
        if any(abs(a - b) > excursion for a, b in zip(start, positions)):
            raise ValueError(
                'Plan exceeds max_joint_excursion; inspect IK/path, do not blindly raise')


def successful_result(wrapped, label):
    """Require both action success and MoveIt success, including execution."""
    if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS):
        raise RuntimeError(f'{label} failed: action={wrapped.status}, '
                           f'MoveIt={wrapped.result.error_code.val}')
    return wrapped.result


class PoseGoalNode(Node):
    """Own bounded action requests, current state and execution cancellation."""

    def __init__(self):
        """Declare safe defaults; position and quaternion have no default goal."""
        super().__init__('ur7e_pose_goal')
        self.values = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        for name in ('position', 'quaternion_xyzw'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
            self.values[name] = self.get_parameter(name).value or []
        self.values = validate_parameters(self.values)
        self.names = []
        self.sample = None
        self.sample_time = 0.0
        self.active_handle = None
        self.result_future = None
        self.create_subscription(JointState, '/joint_states', self.on_state,
                                 qos_profile_sensor_data)
        self.display = self.create_publisher(
            DisplayTrajectory, '/display_planned_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_state(self, message):
        """Accept only a complete finite arm state with a recent ROS timestamp."""
        if not self.names or len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if len(values) != len(message.name) or not set(self.names) <= values.keys():
            return
        if not all(math.isfinite(values[name]) for name in self.names):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -0.1 <= age <= 1.0:
            return
        self.sample = message
        self.sample_time = time.monotonic()

    def current_state(self):
        """Wait for a new complete state, never reuse an unknown startup pose."""
        started = time.monotonic()
        wait_until(self, lambda: self.sample_time > started, 10.0, 'fresh arm joint_states')
        state = RobotState(is_diff=False)
        positions = dict(zip(self.sample.name, self.sample.position))
        state.joint_state = JointState(
            header=copy.deepcopy(self.sample.header), name=self.names,
            position=[positions[name] for name in self.names])
        return state

    def action(self, action_type, name, goal, timeout):
        """Send one action with late-accept cancellation and checked terminal status."""
        client = ActionClient(self, action_type, name)
        try:
            if not client.wait_for_server(timeout_sec=15.0):
                raise TimeoutError(f'Action unavailable: {name}')
            sending = client.send_goal_async(goal)
            try:
                self.active_handle = future_result(self, sending, 10.0, f'{name} acceptance')
            except BaseException:
                # A server may accept after our local deadline: cancel that late goal too.
                def cancel_late(future):
                    handle = future.result()
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                sending.add_done_callback(cancel_late)
                deadline = time.monotonic() + 5.0
                while rclpy.ok() and not sending.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().error(
                    'Action acceptance uncertain. Use pendant Stop if motion is possible.')
                raise
            if not self.active_handle.accepted:
                self.active_handle = None
                raise RuntimeError(f'{name} goal rejected')
            self.result_future = self.active_handle.get_result_async()
            wrapped = future_result(self, self.result_future, timeout, f'{name} result')
            self.active_handle = None
            return successful_result(wrapped, name)
        finally:
            if self.active_handle is not None:
                self.cancel_active()
            client.destroy()

    def cancel_active(self):
        """Request cancellation and wait for termination; never claim a physical stop."""
        try:
            future_result(self, self.active_handle.cancel_goal_async(), 5.0, 'cancel response')
            if self.result_future is not None:
                future_result(self, self.result_future, 5.0, 'action termination')
            self.get_logger().warning('Action terminated after cancellation request.')
        except Exception as error:
            self.get_logger().error(f'Cancellation unconfirmed: {error}. Use pendant Stop.')
        finally:
            self.active_handle = None

    def confirm_review(self):
        """Keep spinning while the local operator reviews this exact displayed plan."""
        if not sys.stdin.isatty():
            raise RuntimeError('Execution requires an interactive terminal for plan review')
        print('\nInspect this NEW plan in RViz. Clear the workspace. '
              'Type EXECUTE then Enter within 120 s to execute it: ', flush=True)
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline().strip() != 'EXECUTE':
                    raise RuntimeError('Execution not confirmed; no trajectory sent')
                return
        raise TimeoutError('Plan review expired; no trajectory sent')

    def run(self):
        """Discover model, check scene/state, plan, display, optionally review and execute."""
        description = read_description(self)
        params = AsyncParameterClient(self, '/move_group')
        if not params.wait_for_services(timeout_sec=15.0):
            raise TimeoutError('move_group parameter services unavailable')
        response = future_result(self, params.get_parameters(['robot_description_semantic']),
                                 10.0, 'MoveIt SRDF')
        group, base, tip, self.names, real = inspect_model(
            description, response.values[0].string_value)
        if self.values['execute'] and not real:
            raise RuntimeError('Real execution requires the official UR hardware plugin')
        self.get_logger().info(
            f'Planning {group}: {base} -> {tip}; velocity={self.values["velocity_scaling"]}, '
            f'acceleration={self.values["acceleration_scaling"]}; '
            f'execute={self.values["execute"]}')
        before = get_scene(self)
        require_table(before, self.values['table_id'])
        state = self.current_state()
        validity = service_call(self, GetStateValidity, '/check_state_validity',
                                GetStateValidity.Request(robot_state=state, group_name=group))
        if not validity.valid:
            raise RuntimeError('Current state is invalid/in collision; inspect scene and robot')
        goal = pose_goal(self.values, group, base, tip, state)
        result = self.action(MoveGroup, '/move_action', goal,
                             self.values['planning_timeout'] + 20.0)
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        arm = {name: current[name] for name in self.names}
        validate_trajectory(result.planned_trajectory, arm, self.values['max_joint_excursion'])
        display = DisplayTrajectory(model_id='ur7e', trajectory_start=result.trajectory_start,
                                    trajectory=[result.planned_trajectory])
        self.display.publish(display)
        self.get_logger().info(
            'Planning SUCCEEDED; trajectory published to /display_planned_path.')
        if not self.values['execute']:
            # Keep the transient publisher alive long enough for RViz discovery/playback.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('PLAN ONLY complete. No execution request was sent.')
            return
        self.confirm_review()
        controllers = service_call(self, ListControllers, '/controller_manager/list_controllers',
                                   ListControllers.Request())
        if not any(c.name == CONTROLLER and c.state == 'active' for c in controllers.controller):
            raise RuntimeError(f'{CONTROLLER} is inactive; check External Control')
        after = get_scene(self)
        require_table(after, self.values['table_id'])
        if (before.world != after.world
                or before.allowed_collision_matrix != after.allowed_collision_matrix):
            raise RuntimeError('Scene changed during review; plan and review again')
        fresh = self.current_state().joint_state
        fresh_map = dict(zip(fresh.name, fresh.position))
        if any(abs(fresh_map[name] - arm[name]) > 0.01 for name in self.names):
            raise RuntimeError('Robot moved during review; plan and review again')
        execute = ExecuteTrajectory.Goal(
            trajectory=result.planned_trajectory, controller_names=[CONTROLLER])
        self.action(ExecuteTrajectory, '/execute_trajectory', execute,
                    self.values['execution_timeout'])
        self.get_logger().info('Execution SUCCEEDED (MoveIt action and error code confirmed).')


def main(args=None):
    """Run one requested pose; all failures return nonzero and release ROS resources."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PoseGoalNode()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning('Interrupted. Use pendant Stop to ensure physical stopping.')
        raise SystemExit(130)
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'UR7e request FAILED: {error}')
        else:
            print(f'UR7e parameters FAILED: {error}', file=sys.stderr)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
