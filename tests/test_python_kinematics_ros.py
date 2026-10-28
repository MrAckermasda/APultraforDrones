"""Real ROS service integration with a simulated bus; no controller/serial I/O."""
from pathlib import Path
import sys
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('kinematics_msgs.srv')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from kinematics_msgs.srv import SetJointValue, SetRobotPose, SetLink, GetLink, GetRobotPose
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import BusServoState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/driver/kinematics'))
from kinematics.python_kinematics_node import PythonKinematicsNode

PULSES = [500, 600, 825, 110, 537]


@pytest.fixture
def ros_system():
    rclpy.init()
    bus = Node('test_kinematics_bus')
    state = {'success': True, 'requests': 0}
    def read(request, response):
        assert [cmd.id for cmd in request.cmd] == [6, 5, 4, 3, 2]
        assert all(cmd.get_position == 1 and cmd.get_id == 1 for cmd in request.cmd)
        state['requests'] += 1
        response.success = state['success']
        for servo_id, pulse in list(zip([6, 5, 4, 3, 2], PULSES))[::-1]:
            value = BusServoState()
            value.present_id, value.position = [servo_id], [pulse]
            response.state.append(value)
        return response
    bus.create_service(GetBusServoState, '/test_kinematics_bus/get_state', read)
    node = PythonKinematicsNode(parameter_overrides=[
        Parameter('enable_pose_targets', value=True),
        Parameter('feedback_service', value='/test_kinematics_bus/get_state'),
        Parameter('feedback_period', value=.05),
        Parameter('feedback_timeout', value=.4),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(bus)
    executor.add_node(node)
    def spin_until(predicate):
        deadline = time.monotonic() + 4.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate(), 'ROS service timed out'
    def call(kind, name, request=None):
        client = bus.create_client(kind, '/kinematics_python/' + name)
        assert client.wait_for_service(timeout_sec=2.)
        future = client.call_async(request or kind.Request())
        spin_until(future.done)
        result = future.result()
        bus.destroy_client(client)
        return result
    try:
        spin_until(lambda: node.feedback.pulses is not None)
        yield node, bus, state, call, spin_until
    finally:
        executor.shutdown()
        node.destroy_node()
        bus.destroy_node()
        rclpy.shutdown()


def test_fk_ik_services_and_measured_id_order(ros_system):
    node, bus, state, call, _ = ros_system
    assert node.feedback.get() == tuple(PULSES)
    assert state['requests'] > 0
    fk_req = SetJointValue.Request()
    fk_req.joint_value = [float(p) for p in PULSES]
    fk = call(SetJointValue, 'set_joint_value_target', fk_req)
    assert fk.success and fk.solution
    measured = call(GetRobotPose, 'get_current_pose')
    assert measured.success and measured.solution
    assert measured.pose == fk.pose
    pose = node.solver.model.fk(node.solver.model.pulses_to_angles(PULSES))
    req = SetRobotPose.Request()
    req.position, req.pitch = pose.position.tolist(), pose.pitch_deg
    req.pitch_range, req.resolution, req.duration = [-90., 90.], 1., .2
    ik = call(SetRobotPose, 'set_pose_target', req)
    assert ik.success and list(ik.pulse) == PULSES
    assert list(ik.current_pulse) == PULSES
    path = call(SetRobotPose, 'set_pose_target_smooth', req)
    assert path.success and list(path.pulse) == PULSES * 11
    assert call(Trigger, 'init_finish').success
    # The solver has no publisher on either motion command topic.
    publishers = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
    assert not any(name in ('/servo_controller', '/ros_robot_controller/bus_servo/set_position')
                   for name, _ in publishers)


def test_invalid_request_does_not_kill_node_and_link_update_is_atomic(ros_system):
    node, _, _, call, _ = ros_system
    req = SetJointValue.Request()
    req.joint_value = [500.]
    response = call(SetJointValue, 'set_joint_value_target', req)
    assert not response.success and not response.solution
    old = call(GetLink, 'get_link').data
    change = SetLink.Request()
    change.data = old
    change.data.link1 = -1.
    assert not call(SetLink, 'set_link', change).success
    assert call(GetLink, 'get_link').data.link1 == .10048
    req.joint_value = [float(p) for p in PULSES]
    assert call(SetJointValue, 'set_joint_value_target', req).success


def test_failed_measurements_clear_seed_and_gate_pose_outputs(ros_system):
    node, _, state, call, spin_until = ros_system
    state['success'] = False
    spin_until(lambda: node.feedback.pulses is None)
    assert not call(GetRobotPose, 'get_current_pose').success
    req = SetRobotPose.Request()
    req.position, req.pitch_range, req.resolution = [.2, 0., .2], [-90., 90.], 1.
    failed = call(SetRobotPose, 'set_pose_target', req)
    assert not failed.success and not failed.pulse
    assert not call(Trigger, 'init_finish').success
    state['success'] = True
    spin_until(lambda: node.feedback.pulses is not None)
    node.enabled = False
    disabled = call(SetRobotPose, 'set_pose_target', req)
    assert not disabled.success and not disabled.pulse


def test_default_configuration_disables_pose_targets():
    rclpy.init()
    node = PythonKinematicsNode()
    try:
        assert node.enabled is False
        node.feedback.update(zip([6, 5, 4, 3, 2], PULSES))
        response = node.solve_pose(SetRobotPose.Request(), SetRobotPose.Response())
        assert not response.success and not response.pulse
    finally:
        node.destroy_node()
        rclpy.shutdown()
