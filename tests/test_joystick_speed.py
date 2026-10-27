"""Offline gamepad command tests: no ROS node, serial port or gamepad is opened."""
import importlib.util
from pathlib import Path
import runpy
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/peripherals/peripherals/joystick_control.py'
LAUNCH = ROOT / 'src/peripherals/launch/joystick_control.launch.py'
DEFAULTS = dict(joint_step=10, joint_duration=0.05, cartesian_step=0.005,
                cartesian_duration=0.02, pitch_step=1.0)


@pytest.fixture(scope='module')
def module():
    spec = importlib.util.spec_from_file_location('joystick_speed_under_test', SOURCE)
    module = importlib.util.module_from_spec(spec)
    # Imports should only define interfaces, never connect to hardware.
    with patch('serial.Serial', side_effect=AssertionError('No hardware in tests')):
        spec.loader.exec_module(module)
    return module


def make_controller(module, overrides=None, declare=True):
    # Deliberately bypass __init__: it waits for services and commands a pose.
    node = object.__new__(module.JoystickController)
    overrides = overrides or {}
    node.declare_parameter = Mock(side_effect=lambda name, default, **kwargs:
                                  SimpleNamespace(value=overrides.get(name, default)))
    node.get_logger = Mock(return_value=Mock())
    node.add_on_set_parameters_callback = Mock()
    node.add_post_set_parameters_callback = Mock()
    if declare:
        node._declare_jog_parameters()
    node.mode = 0
    node.current_servo_position = np.array([500] * 6)
    node.servos_pub = Mock()
    node.buzzer_pub = Mock()
    node.run_async = Mock()
    return node


def test_defaults_and_read_only_descriptors(module):
    node = make_controller(module)
    assert {name: getattr(node, name) for name in DEFAULTS} == DEFAULTS
    assert len(node.declare_parameter.call_args_list) == len(DEFAULTS)
    for call in node.declare_parameter.call_args_list:
        assert call.kwargs['descriptor'].read_only is (call.args[0] not in ('joint_step', 'joint_duration'))


@pytest.mark.parametrize('name,value', [
    ('joint_step', 0), ('joint_step', 21), ('joint_step', 5.0),
    ('joint_step', True), ('joint_step', '5'),
    ('joint_duration', 0.0), ('joint_duration', 0.019),
    ('joint_duration', 30.1), ('joint_duration', float('nan')),
    ('joint_duration', float('inf')), ('joint_duration', 1),
    ('cartesian_step', -0.001), ('cartesian_step', 0.0101),
    ('cartesian_duration', 0.019), ('cartesian_duration', float('-inf')),
    ('pitch_step', 0.0), ('pitch_step', 2.1),
])
def test_invalid_settings_rejected(module, name, value):
    with pytest.raises(ValueError, match=name):
        make_controller(module, {name: value})


@pytest.mark.parametrize('name,lower,upper', [
    ('joint_step', 1, 20), ('joint_duration', 0.02, 30.0),
    ('cartesian_step', 0.0001, 0.01),
    ('cartesian_duration', 0.02, 30.0), ('pitch_step', 0.1, 2.0),
])
def test_parameter_boundaries(module, name, lower, upper):
    for value in (lower, upper):
        node = make_controller(module, {name: value})
        assert getattr(node, name) == value


def test_invalid_startup_fails_before_io_or_movement(module):
    node = make_controller(module, {'joint_step': 0}, declare=False)
    node.create_publisher = Mock()
    with patch.object(module.rclpy, 'init'), \
         patch.object(module.Node, '__init__', return_value=None), \
         patch.object(module.pg.display, 'init') as display, \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        with pytest.raises(ValueError, match='joint_step'):
            module.JoystickController.__init__(node, 'joystick_control')
        display.assert_not_called()
        node.create_publisher.assert_not_called()
        send.assert_not_called()


# (input, sign of input, index in cached positions, servo ID, direction)
JOINT_INPUTS = [
    ('hat_x', 1, 1, 5, 1), ('hat_x', -1, 1, 5, -1),
    ('hat_y', 1, 0, 6, -1), ('hat_y', -1, 0, 6, 1),
    ('l1', 1, 5, 1, -1), ('l2', 1, 5, 1, 1),
    ('r1', 1, 3, 3, 1), ('r2', 1, 3, 3, -1),
    ('square', 1, 4, 2, 1), ('circle', 1, 4, 2, -1),
    ('cross', 1, 2, 4, -1), ('triangle', 1, 2, 4, 1),
]


def press(node, key, value):
    if key.startswith('hat_'):
        axes = dict(hat_x=0, hat_y=0)
        axes[key] = value
        node.axes_callback(axes)
    else:
        getattr(node, key + '_callback')(bool(value))


@pytest.mark.parametrize('step,duration', [(10, 0.05), (5, 0.1), (12, 0.05)])
@pytest.mark.parametrize('key,value,index,servo,direction', JOINT_INPUTS)
def test_joint_step_duration_and_original_mapping(
        module, step, duration, key, value, index, servo, direction):
    node = make_controller(module, {'joint_step': step, 'joint_duration': duration})
    with patch.object(module.bus_servo_control, 'set_servo_position') as send:
        press(node, key, value)
        target = 500 + direction * step
        send.assert_called_once_with(node.servos_pub, duration, ((servo, target),))
        expected = [500] * 6
        expected[index] = target
        assert list(node.current_servo_position) == expected
        send.reset_mock()
        # Release itself must not send a new command. Existing event-loop
        # repeat behavior is deliberately outside this parameter-only change.
        press(node, key, 0)
        send.assert_not_called()


@pytest.mark.parametrize('key,value,index,servo,direction', JOINT_INPUTS)
def test_no_joint_command_without_state(module, key, value, index, servo, direction):
    node = make_controller(module)
    node.current_servo_position = None
    with patch.object(module.bus_servo_control, 'set_servo_position') as send:
        press(node, key, value)
        send.assert_not_called()


CARTESIAN_INPUTS = [
    ('hat_x', 1, (1, 0, 0, 0)), ('hat_x', -1, (-1, 0, 0, 0)),
    ('hat_y', 1, (0, -1, 0, 0)), ('hat_y', -1, (0, 1, 0, 0)),
    ('l1', 1, (0, 0, 1, 0)), ('l2', 1, (0, 0, -1, 0)),
    ('r1', 1, (0, 0, 0, -1)), ('r2', 1, (0, 0, 0, 1)),
]


@pytest.mark.parametrize('step,pitch', [(0.005, 1.0), (0.002, 0.5)])
@pytest.mark.parametrize('key,value,directions', CARTESIAN_INPUTS)
def test_cartesian_steps_keep_directions(module, step, pitch, key, value, directions):
    node = make_controller(module, {'cartesian_step': step, 'pitch_step': pitch})
    node.mode = 1
    node.relative_move_async = Mock()
    press(node, key, value)
    increments = tuple(d * (pitch if i == 3 else step)
                       for i, d in enumerate(directions))
    node.relative_move_async.assert_called_once_with(*increments)
    node.relative_move_async.reset_mock()
    press(node, key, 0)
    node.relative_move_async.assert_not_called()


@pytest.mark.parametrize('duration', [0.02, 0.1])
@pytest.mark.parametrize('reachable', [True, False])
def test_ik_result_uses_cartesian_duration(module, duration, reachable):
    node = make_controller(module, {'cartesian_duration': duration})
    node.get_current_pose_client = Mock()
    node.kinematics_client = Mock()
    endpoint = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.0, z=0.2), orientation=Mock()))
    result = SimpleNamespace(pulse=[100, 200, 300, 400] if reachable else [])
    node.send_request = Mock(side_effect=[endpoint, result])
    with patch.object(module.transform, 'qua2rpy', return_value=(0.0, 0.0, 0.0)), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.relative_move_async(0.002, 0, 0, 0.5)
        if reachable:
            send.assert_called_once_with(node.servos_pub, duration,
                                         ((3, 400), (4, 300), (5, 200), (6, 100)))
        else:
            send.assert_not_called()


def test_start_home_duration_unchanged(module):
    node = make_controller(module, {'joint_step': 5, 'joint_duration': 0.1})
    node.start_callback(True)
    node.run_async.assert_called_once_with(module.actions.go_home, node.servos_pub, 1.0)
    node.run_async.reset_mock()
    node.start_callback(False)
    node.run_async.assert_not_called()


@pytest.mark.parametrize('overrides', [
    {},
    dict(joint_step='5', joint_duration='0.1', cartesian_step='0.002',
         cartesian_duration='0.1', pitch_step='0.5'),
])
def test_launch_parameters_match_node_types_and_values(overrides):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    # Construct declarations only: no launch service and no executable starts.
    with patch('launch_ros.actions.Node') as node_class:
        description = runpy.run_path(str(LAUNCH))['generate_launch_description']()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    arm_call = next(call for call in node_class.call_args_list
                    if call.kwargs['executable'] == 'joystick_control')
    parameters = evaluate_parameters(context, normalize_parameters(arm_call.kwargs['parameters']))
    actual = parameters[-1]
    for name, default in DEFAULTS.items():
        expected = type(default)(overrides[name]) if name in overrides else default
        assert actual[name] == expected
        assert type(actual[name]) is type(default)
