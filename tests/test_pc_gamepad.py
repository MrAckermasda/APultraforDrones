"""PC gamepad startup/event regressions. No constructor touches real hardware."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import runpy

import pytest

from test_joystick_speed import module, make_controller, LAUNCH


@pytest.mark.parametrize('enable', [False, True])
def test_constructor_waits_only_for_servo_manager(module, enable):
    node = object.__new__(module.JoystickController)
    node.declare_parameter = Mock(side_effect=lambda name, default, **kw:
                                  SimpleNamespace(value=enable if name == 'enable_kinematics' else default))
    node.get_logger = Mock(return_value=Mock())
    node.add_on_set_parameters_callback = Mock()
    node.add_post_set_parameters_callback = Mock()
    node.create_publisher = Mock()
    node.create_subscription = Mock()
    node.create_timer = Mock()
    node.create_client = Mock(side_effect=lambda *args, **kw: Mock())
    with patch.object(module.rclpy, 'init'), patch.object(module.Node, '__init__', return_value=None), \
         patch.object(module.pg.display, 'init'), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        module.JoystickController.__init__(node, 'joystick_control')
    client_names = [call.args[1] for call in node.create_client.call_args_list]
    assert client_names[-1] == '/controller_manager/init_finish'
    assert len(client_names) == (3 if enable else 1)
    node.client.wait_for_service.assert_called_once_with(timeout_sec=2.0)
    if enable:
        node.get_current_pose_client.wait_for_service.assert_not_called()
        node.kinematics_client.wait_for_service.assert_not_called()
    else:
        assert node.get_current_pose_client is None and node.kinematics_client is None
    assert node.create_timer.call_count == 2
    send.assert_called_once_with(node.servos_pub, 1.0,
                                ((6, 500), (5, 640), (4, 825), (3, 110), (2, 500), (1, 200)))


def input_node(module):
    node = make_controller(module)
    node.enable_kinematics = False
    node.js = Mock(spec_set=module.pg.joystick.JoystickType)
    node.js.get_instance_id.return_value = 7
    node.js.get_init.return_value = True
    node.BUTTONS = module.BUTTONS[1]
    node.last_axes = dict.fromkeys(module.AXES_MAP, 0.0)
    node.last_buttons = [0] * len(node.BUTTONS)
    return node


def test_one_event_batch_returns_and_preserves_release(module):
    node = input_node(module)
    events = [SimpleNamespace(type=module.pg.JOYBUTTONDOWN, instance_id=7, button=4),
              SimpleNamespace(type=module.pg.JOYBUTTONUP, instance_id=7, button=4)]
    with patch.object(module.pg.event, 'get', return_value=events) as poll, \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    poll.assert_called_once()
    send.assert_called_once_with(node.servos_pub, 0.05, ((1, 490),))
    assert node.last_buttons[4] == 0


def test_hat_does_not_repeat_on_unrelated_events_or_release(module):
    node = input_node(module)
    events = [SimpleNamespace(type=module.pg.JOYHATMOTION, instance_id=7, hat=0, value=(0, 1)),
              SimpleNamespace(type=module.pg.JOYAXISMOTION, instance_id=7, axis=0, value=0.5),
              SimpleNamespace(type=module.pg.JOYBUTTONUP, instance_id=7, button=4),
              SimpleNamespace(type=module.pg.JOYHATMOTION, instance_id=7, hat=0, value=(0, 0))]
    with patch.object(module.pg.event, 'get', return_value=events), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    send.assert_called_once_with(node.servos_pub, 0.05, ((5, 510),))
    assert node.last_axes['hat_x'] == node.last_axes['hat_y'] == 0


@pytest.mark.parametrize('disconnected', [False, True])
def test_disconnect_discards_batch_and_clears_state(module, disconnected):
    node = input_node(module)
    joystick = node.js
    node.js.get_init.return_value = not disconnected
    node.last_axes['hat_x'] = 1
    events = [SimpleNamespace(type=module.pg.JOYBUTTONDOWN, instance_id=7, button=4),
              SimpleNamespace(type=module.pg.JOYDEVICEREMOVED, instance_id=7)]
    with patch.object(module.pg.event, 'get', return_value=events), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    send.assert_not_called()
    joystick.quit.assert_called_once()
    assert node.js is None and not any(node.last_axes.values())


def test_unsupported_indices_and_other_gamepad_are_ignored(module):
    node = input_node(module)
    events = [SimpleNamespace(type=module.pg.JOYBUTTONDOWN, instance_id=8, button=4),
              SimpleNamespace(type=module.pg.JOYBUTTONDOWN, instance_id=7, button=99),
              SimpleNamespace(type=module.pg.JOYAXISMOTION, instance_id=7, axis=99, value=1),
              SimpleNamespace(type=module.pg.JOYHATMOTION, instance_id=7, hat=1, value=(0, 1))]
    with patch.object(module.pg.event, 'get', return_value=events), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    send.assert_not_called()


def test_no_gamepad_discards_events(module):
    node = input_node(module)
    node.js = None
    with patch.object(module.pg.event, 'get') as poll, patch.object(module.pg.event, 'clear') as clear:
        node.update_buttons()
    clear.assert_called_once()
    poll.assert_not_called()


@pytest.mark.parametrize('name,layout', [('USB WirelessGamepad ', 1), ('SHANWAN Android Gamepad', 0)])
def test_connection_normalizes_name_and_clears_old_input(module, name, layout):
    node = input_node(module)
    node.js = None
    candidate = Mock(spec_set=module.pg.joystick.JoystickType)
    candidate.get_name.return_value = name
    with patch.object(module.pg.joystick, 'init'), \
         patch.object(module.pg.joystick, 'get_count', return_value=1), \
         patch.object(module.pg.joystick, 'Joystick', return_value=candidate), \
         patch.object(module.pg.event, 'clear') as clear:
        node.connect()
    assert node.js is candidate and node.BUTTONS == module.BUTTONS[layout]
    clear.assert_called_once()


def test_unknown_gamepad_is_not_assigned_guessed_buttons(module):
    node = input_node(module)
    node.js = None
    candidate = Mock(spec_set=module.pg.joystick.JoystickType)
    candidate.get_name.return_value = 'Different controller'
    with patch.object(module.pg.joystick, 'init'), \
         patch.object(module.pg.joystick, 'get_count', return_value=1), \
         patch.object(module.pg.joystick, 'Joystick', return_value=candidate):
        node.connect()
    assert node.js is None
    candidate.quit.assert_called_once()


def test_state_uses_ids_not_message_order(module):
    node = input_node(module)
    states = [SimpleNamespace(id=i, position=i * 100) for i in (1, 4, 2, 6, 3, 5)]
    node.servo_states_callback(SimpleNamespace(servo_state=states))
    assert list(node.current_servo_position) == [600, 500, 400, 300, 200, 100]
    node.servo_states_callback(SimpleNamespace(servo_state=states[:-1]))
    assert node.current_servo_position is None


@pytest.mark.parametrize('enable,ready,mode', [(False, True, 0), (True, False, 0), (True, True, 1)])
def test_cartesian_mode_requires_opt_in_and_services(module, enable, ready, mode):
    node = input_node(module)
    node.enable_kinematics = enable
    node.get_current_pose_client = Mock()
    node.kinematics_client = Mock()
    node.get_current_pose_client.service_is_ready.return_value = ready
    node.kinematics_client.service_is_ready.return_value = ready
    node.mode_callback(True)
    assert node.mode == mode
    if mode:
        node.mode_callback(True)
        assert node.mode == 0
    else:
        node.buzzer_pub.publish.assert_not_called()


def test_service_timeout_cancels_and_returns(module):
    node = input_node(module)
    client = Mock()
    with patch.object(module.threading, 'Event') as event:
        event.return_value.wait.return_value = False
        assert node.send_request(client, Mock()) is None
    event.return_value.wait.assert_called_once_with(timeout=1.0)
    client.call_async.return_value.cancel.assert_called_once()


@pytest.mark.parametrize('results', [[None], [SimpleNamespace(pose=SimpleNamespace(
    position=SimpleNamespace(x=0, y=0, z=0), orientation=Mock())), None]])
def test_missing_kinematics_reply_never_moves(module, results):
    node = input_node(module)
    node.get_current_pose_client = Mock()
    node.kinematics_client = Mock()
    node.send_request = Mock(side_effect=results)
    with patch.object(module.transform, 'qua2rpy', return_value=(0, 0, 0)), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.relative_move_async(0.001, 0, 0, 0)
    send.assert_not_called()


def test_launch_starts_only_pygame_controller_and_disables_ik_by_default():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    context = LaunchContext()
    with patch('launch_ros.actions.Node') as node:
        description = runpy.run_path(str(LAUNCH))['generate_launch_description']()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert node.call_count == 1
    assert node.call_args.kwargs['executable'] == 'joystick_control'
    values = evaluate_parameters(context, normalize_parameters(node.call_args.kwargs['parameters']))
    assert values[-2]['enable_kinematics'] is False


def test_connected_poll_uses_actual_pygame_joystick_api(module):
    node = input_node(module)
    # An unrestricted Mock previously hid nonexistent Joystick methods.
    with pytest.raises(AttributeError):
        node.js.get_attached()
    with patch.object(module.pg.joystick, 'Joystick') as create, \
         patch.object(module.pg.event, 'get', return_value=[]), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.connect()
        node.update_buttons()
        node.connect()
    create.assert_not_called()
    send.assert_not_called()


def test_other_device_removal_keeps_active_gamepad(module):
    node = input_node(module)
    joystick = node.js
    events = [SimpleNamespace(type=module.pg.JOYDEVICEREMOVED, instance_id=99)]
    with patch.object(module.pg.event, 'get', return_value=events), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    assert node.js is joystick
    joystick.quit.assert_not_called()
    send.assert_not_called()


def test_pygame_read_error_disconnects_without_motion(module):
    node = input_node(module)
    with patch.object(module.pg.event, 'get', side_effect=module.pg.error('removed')), \
         patch.object(module.bus_servo_control, 'set_servo_position') as send:
        node.update_buttons()
    assert node.js is None
    send.assert_not_called()
