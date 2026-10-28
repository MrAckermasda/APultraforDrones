"""Portable ROS services using measured bus feedback; never publishes motion."""
import math
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from kinematics_msgs.srv import (SetRobotPose, SetJointValue, GetRobotPose,
                                 SetLink, GetLink, SetJointRange, GetJointRange)
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState

from .python_kinematics import ArmGeometry, ArmKinematics, NOMINAL_LIMITS_DEG, SERVO_IDS
from .deployment import DeploymentSolver, Feedback, rotation_rpy_deg


class PythonKinematicsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('kinematics_python', **kwargs)
        geometry = ArmGeometry.for_chassis(os.environ.get('CHASSIS_TYPE', 'None'))
        defaults = {
            'enable_pose_targets': False,
            'zero_pulses': [500.] * 5,
            'joint_limits_deg': [float(v) for pair in NOMINAL_LIMITS_DEG for v in pair],
            'feedback_timeout': 1.0,
            'feedback_period': 0.2,
            'feedback_service': '/ros_robot_controller/bus_servo/get_state',
            'position_tolerance': 0.002,
            'pitch_tolerance': 0.5,
            'max_velocity': 250.,
            'max_acceleration': 5000.,
            'max_samples': 501,
            'trajectory_dt': 0.02,
        }
        defaults.update({name: getattr(geometry, name) for name in geometry.__dataclass_fields__})
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        geometry = ArmGeometry(**{name: param(name) for name in geometry.__dataclass_fields__})
        model = ArmKinematics(geometry, np.asarray(param('joint_limits_deg')).reshape(5, 2),
                              param('zero_pulses'))
        self.solver = DeploymentSolver(model, **{name: param(name) for name in (
            'position_tolerance', 'pitch_tolerance', 'max_velocity', 'max_acceleration', 'max_samples')})
        self.feedback = Feedback(model, param('feedback_timeout'))
        self.enabled = param('enable_pose_targets')
        self.dt = param('trajectory_dt')
        period = param('feedback_period')
        if not math.isfinite(self.dt) or self.dt <= 0 or not math.isfinite(period) or period <= 0:
            raise ValueError('trajectory_dt and feedback_period must be finite and positive')
        self.pending = None
        self.request_started = None
        self.bus = self.create_client(GetBusServoState, param('feedback_service'))
        self.create_timer(period, self.poll_feedback)
        services = (
            (SetRobotPose, 'set_pose_target', self.solve_pose),
            (SetRobotPose, 'set_pose_target_smooth', self.solve_trajectory),
            (SetJointValue, 'set_joint_value_target', self.solve_fk),
            (GetRobotPose, 'get_current_pose', self.current_pose),
            (SetLink, 'set_link', self.set_link),
            (GetLink, 'get_link', self.get_link),
            (SetJointRange, 'set_joint_range', self.set_limits),
            (GetJointRange, 'get_joint_range', self.get_limits),
            (Trigger, 'init_finish', self.ready),
        )
        for kind, name, callback in services:
            self.create_service(kind, '~/' + name, callback)
        self.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
            successful=False, reason='Restart with parameters/YAML to change configuration'))
        self.get_logger().info(
            f'Python FK/IK ready; pose targets enabled={self.enabled}; '
            'uses measured bus positions; no motion publisher')

    def poll_feedback(self):
        if self.pending is not None:
            if time.monotonic() - self.request_started > self.feedback.timeout:
                future, self.pending = self.pending, None
                self.feedback.invalidate()
                self.bus.remove_pending_request(future)
                future.cancel()
            return
        if not self.bus.service_is_ready():
            self.feedback.invalidate()
            return
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_id = 1
            command.get_position = 1
            request.cmd.append(command)
        self.request_started = time.monotonic()
        self.pending = self.bus.call_async(request)
        self.pending.add_done_callback(self.receive_feedback)

    def receive_feedback(self, future):
        if future is not self.pending:
            return
        self.pending = None
        try:
            response = future.result()
            if response is None or not response.success or len(response.state) != 5:
                raise ValueError('Bus feedback failed or incomplete')
            pairs = []
            for state in response.state:
                if len(state.present_id) != 1 or len(state.position) != 1:
                    raise ValueError('Missing servo ID or measured position')
                pairs.append((state.present_id[0], state.position[0]))
            # Timestamp at request START: sequential bus reads are not simultaneous.
            self.feedback.update(pairs, self.request_started)
        except Exception as exc:
            self.feedback.invalidate()
            self.get_logger().warning(f'Measured feedback rejected: {exc}')

    def ready(self, request, response):
        try:
            self.require_feedback()
            response.success = True
            response.message = 'Pose targets enabled; measured joint positions available'
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def require_feedback(self):
        if not self.enabled:
            raise ValueError('Pose targets disabled; validate geometry and calibration before enabling')
        return self.feedback.get()

    @staticmethod
    def fill_pose(response, pose):
        response.pose.position.x, response.pose.position.y, response.pose.position.z = map(float, pose.position)
        q = response.pose.orientation
        q.x, q.y, q.z, q.w = map(float, pose.quaternion_xyzw)
        response.success = True
        response.solution = True
        return response

    def fail(self, response, exc):
        response.success = False
        if hasattr(response, 'solution'):
            response.solution = False
        if hasattr(response, 'pulse'):
            response.pulse = []
            response.current_pulse = []
            response.rpy = []
            response.min_variation = 0.
        if hasattr(response, 'message'):
            response.message = str(exc)
        self.get_logger().warning(str(exc))
        return response

    def solve_fk(self, request, response):
        try:
            model = self.solver.model
            return self.fill_pose(response, model.fk(model.pulses_to_angles(request.joint_value)))
        except ValueError as exc:
            return self.fail(response, exc)

    def current_pose(self, request, response):
        try:
            model = self.solver.model
            return self.fill_pose(response, model.fk(model.pulses_to_angles(self.feedback.get())))
        except ValueError as exc:
            return self.fail(response, exc)

    def solve_pose(self, request, response):
        return self._solve(request, response, smooth=False)

    def solve_trajectory(self, request, response):
        return self._solve(request, response, smooth=True)

    def _solve(self, request, response, smooth):
        try:
            current = self.require_feedback()
            args = (request.position, request.pitch, request.pitch_range, request.resolution, current)
            if smooth:
                path, result = self.solver.trajectory(*args, request.duration, self.dt)
                pulses = [p for point in path for p in point]
                final_pose = self.solver.model.fk(self.solver.model.pulses_to_angles(path[-1]))
            else:
                result = self.solver.solve(*args)
                pulses, final_pose = list(result.pulses), result.pose
            self.feedback.get()  # A long search must not return an expired seed.
            response.success = True
            response.pulse = pulses
            response.current_pulse = list(current)
            response.rpy = rotation_rpy_deg(final_pose.rotation)
            response.min_variation = float(sum(abs(p-c) for p, c in zip(pulses[-5:], current)))
            return response
        except ValueError as exc:
            return self.fail(response, exc)

    def replace_model(self, model):
        self.solver.model = model
        self.enabled = False
        self.feedback.model = model
        self.feedback.invalidate()
        # Reject an in-flight snapshot taken before the configuration change.
        if self.pending is not None:
            future, self.pending = self.pending, None
            self.bus.remove_pending_request(future)
            future.cancel()

    def set_link(self, request, response):
        try:
            d, old = request.data, self.solver.model
            geometry = ArmGeometry(d.base_link, d.link1, d.link2, d.link3, d.end_effector_link)
            self.replace_model(ArmKinematics(geometry, np.degrees(old.limits), old.zero_pulses))
            response.success, response.message = True, 'Link geometry updated; pose targets disabled until restart with validated configuration'
            return response
        except ValueError as exc:
            return self.fail(response, exc)

    def get_link(self, request, response):
        g = self.solver.model.geometry
        d = response.data
        d.base_link, d.link1, d.link2 = g.base_height, g.upper_arm, g.forearm
        d.link3, d.end_effector_link = g.wrist_length, g.gripper_length
        response.success = True
        return response

    def set_limits(self, request, response):
        try:
            limits = [[getattr(request.data, f'joint{i}').min,
                       getattr(request.data, f'joint{i}').max] for i in range(1, 6)]
            old = self.solver.model
            self.replace_model(ArmKinematics(old.geometry, limits, old.zero_pulses))
            response.success, response.message = True, 'Joint limits updated; pose targets disabled until restart with validated configuration'
            return response
        except ValueError as exc:
            return self.fail(response, exc)

    def get_limits(self, request, response):
        for i, (lower, upper) in enumerate(np.degrees(self.solver.model.limits), 1):
            joint = getattr(response.data, f'joint{i}')
            joint.min, joint.max = float(lower), float(upper)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PythonKinematicsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal signal may also be forwarded by ros2 launch. Do not let
        # the second SIGINT interrupt service/client destruction.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
