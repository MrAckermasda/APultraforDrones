"""Real parameter transactions and GUI client tests, never construct a robot driver."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/peripherals'))
from peripherals.jog_settings import JogParameterClient, parse_joint_settings
from test_joystick_speed import module


@pytest.fixture
def parameter_node(module):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    context = Context()
    rclpy.init(context=context)
    # Bypass JoystickController.__init__: no pygame, services, timers or pose.
    node = object.__new__(module.JoystickController)
    Node.__init__(node, 'offline_jog_parameter_test', context=context,
                  start_parameter_services=True, enable_rosout=False)
    node._declare_jog_parameters()
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def test_atomic_apply_updates_future_jog_without_sending_motion(parameter_node, module):
    from rclpy.parameter import Parameter
    import numpy as np
    node = parameter_node
    with patch.object(module.bus_servo_control, 'set_servo_position') as send:
        result = node.set_parameters_atomically([
            Parameter('joint_step', value=7), Parameter('joint_duration', value=0.2)])
        assert result.successful
        assert (node.joint_step, node.joint_duration) == (7, 0.2)
        assert node.get_parameter('joint_step').value == 7
        send.assert_not_called()
        node.mode = 0
        node.current_servo_position = np.array([500] * 6)
        node.servos_pub = Mock()
        node.l1_callback(True)
        send.assert_called_once_with(node.servos_pub, 0.2, ((1, 493),))


@pytest.mark.parametrize('duration', [0.0, float('nan'), float('inf')])
def test_invalid_atomic_pair_keeps_both_old_values(parameter_node, duration):
    from rclpy.parameter import Parameter
    node = parameter_node
    result = node.set_parameters_atomically([
        Parameter('joint_step', value=8), Parameter('joint_duration', value=duration)])
    assert not result.successful
    assert (node.joint_step, node.joint_duration) == (10, 0.05)
    assert node.get_parameter('joint_step').value == 10


def test_other_validator_rejection_does_not_change_cached_settings(parameter_node):
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    node = parameter_node
    node.add_on_set_parameters_callback(lambda _: SetParametersResult(successful=False, reason='rejected'))
    result = node.set_parameters_atomically([Parameter('joint_step', value=9)])
    assert not result.successful and node.joint_step == 10


def test_cartesian_settings_remain_readonly(parameter_node):
    from rclpy.parameter import Parameter
    result = parameter_node.set_parameters_atomically([Parameter('cartesian_step', value=0.002)])
    assert not result.successful
    assert parameter_node.cartesian_step == 0.005


@pytest.mark.parametrize('step,duration', [('5.0', '0.1'), ('0', '0.1'), ('21', '0.1'),
                                        ('5', 'nan'), ('5', 'inf'), ('5', '0.019'),
                                        ('5', '31'), ('9' * 400, '0.1')])
def test_invalid_gui_input_rejected(step, duration):
    with pytest.raises(ValueError):
        parse_joint_settings(step, duration)


def client_fixture():
    node = Mock()
    get_client, set_client = Mock(), Mock()
    node.create_client.side_effect = [get_client, set_client]
    notify = Mock()
    client = JogParameterClient(node, '/joystick_control', notify)
    return client, get_client, set_client, notify


def test_client_sends_atomic_typed_pair_waits_for_confirmation():
    from rcl_interfaces.srv import SetParametersAtomically
    from rcl_interfaces.msg import ParameterType, SetParametersResult
    client, _, remote, notify = client_fixture()
    future = remote.call_async.return_value
    future.done.return_value = False
    values = parse_joint_settings('5', '0.1')
    assert client.request('apply', values)
    request = remote.call_async.call_args.args[0]
    assert isinstance(request, SetParametersAtomically.Request)
    assert [p.value.type for p in request.parameters] == [ParameterType.PARAMETER_INTEGER,
                                                        ParameterType.PARAMETER_DOUBLE]
    client.poll()
    notify.assert_not_called()
    assert not client.request('read')  # don't overlap an outstanding transaction
    future.done.return_value = True
    future.result.return_value = SetParametersAtomically.Response(result=SetParametersResult(successful=True))
    client.poll()
    assert notify.call_args.args[:3] == ('apply', True, values)
    assert client.pending is None


def test_client_read_current_values():
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter
    client, remote, _, notify = client_fixture()
    remote.call_async.return_value.result.return_value = GetParameters.Response(values=[
        Parameter('joint_step', value=6).get_parameter_value(),
        Parameter('joint_duration', value=0.3).get_parameter_value()])
    client.request('read')
    client.poll()
    assert notify.call_args.args[:3] == ('read', True, {'joint_step': 6, 'joint_duration': 0.3})


def test_old_readonly_node_reports_rejection():
    client, _, remote, notify = client_fixture()
    remote.call_async.return_value.result.return_value = SimpleNamespace(
        result=SimpleNamespace(successful=False, reason='read-only'))
    client.request('apply', parse_joint_settings('5', '0.1'))
    client.poll()
    assert notify.call_args.args[1] is False
    assert 'read-only' in notify.call_args.args[3]


def test_service_missing_does_not_send_request():
    client, _, remote, notify = client_fixture()
    remote.service_is_ready.return_value = False
    assert not client.request('apply', parse_joint_settings('5', '0.1'))
    remote.call_async.assert_not_called()
    assert notify.call_args.args[1] is False


def test_timeout_does_not_claim_rejection_or_retry():
    client, _, remote, notify = client_fixture()
    remote.call_async.return_value.done.return_value = False
    with patch('peripherals.jog_settings.time.monotonic', return_value=0):
        client.request('apply', parse_joint_settings('5', '0.1'))
    with patch('peripherals.jog_settings.time.monotonic', return_value=4):
        client.poll()
    remote.call_async.assert_called_once()
    remote.call_async.return_value.cancel.assert_called_once()
    assert client.pending is None
    assert '可能已应用' in notify.call_args.args[3]


def test_gui_parameter_editor_validation_and_acknowledgment(tmp_path):
    import tkinter as tk
    from peripherals.gamepad_monitor import MonitorWindow
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('No display')
    root.withdraw()
    app = MonitorWindow(root, '/fake/js0', tmp_path)
    app.control = Mock(target='/joystick_control')
    try:
        app.step_text.set('21')
        app.request_jog('apply')
        app.control.request.assert_not_called()
        app.step_text.set('5')
        app.duration_text.set('0.1')
        app.request_jog('apply')
        app.control.request.assert_called_once_with('apply', {'joint_step': 5, 'joint_duration': 0.1})
        assert '未读取' in app.jog_current.get()
        app.on_jog_result('apply', True, {'joint_step': 5, 'joint_duration': 0.1}, '已应用')
        assert 'joint_step=5' in app.jog_current.get()
        assert all(str(widget['state']) == 'normal' for widget in app.jog_widgets)
        app.on_jog_result('apply', False, {}, '超时')
        assert '未确认' in app.jog_current.get()
        assert '"source": "settings"' in app.recorder.path.read_text()
    finally:
        app.close()


def test_real_parameter_service_roundtrip_on_non_robot_node(parameter_node):
    import time
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    peer = Node('offline_jog_client_test', context=parameter_node.context,
                start_parameter_services=False, enable_rosout=False)
    executor = SingleThreadedExecutor(context=parameter_node.context)
    executor.add_node(parameter_node)
    executor.add_node(peer)
    results = []
    client = JogParameterClient(peer, '/offline_jog_parameter_test', lambda *args: results.append(args))
    try:
        assert client.get_client.wait_for_service(timeout_sec=2)
        assert client.set_client.wait_for_service(timeout_sec=2)
        for action, values in [('read', None), ('apply', {'joint_step': 4, 'joint_duration': 0.15})]:
            assert client.request(action, values)
            deadline = time.monotonic() + 3
            while client.pending and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                client.poll()
            assert client.pending is None
            assert results[-1][0:2] == (action, True)
        assert (parameter_node.joint_step, parameter_node.joint_duration) == (4, 0.15)
    finally:
        client.close()
        executor.shutdown()
        peer.destroy_node()
