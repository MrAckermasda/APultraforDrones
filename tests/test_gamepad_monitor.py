"""Monitor regression tests: no real ROS publishers or motor device access."""
import csv
import errno
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/peripherals'))
from peripherals.gamepad_monitor_data import (EventRecorder, JoystickReader,
                                              decode_joystick_event, servo_command_data)


def packet(value=1, kind=1, index=2):
    return struct.pack('<IhBB', 1234, value, kind, index)


def test_initial_snapshot_is_not_live_input():
    data = decode_joystick_event(packet(kind=0x81))
    assert data == {'device_ms': 1234, 'kind': 'button', 'index': 2,
                    'value': 1, 'normalized': None, 'initial': True}
    assert not decode_joystick_event(packet())['initial']


@pytest.mark.parametrize('value,normalized', [(-32768, -1), (0, 0), (32767, 1)])
def test_axis_signed_range(value, normalized):
    assert decode_joystick_event(packet(value=value, kind=2))['normalized'] == normalized


def test_reject_incomplete_event():
    with pytest.raises(ValueError):
        decode_joystick_event(b'123')


def test_reader_nonblocking_disconnect_and_reconnect():
    emit = Mock()
    reader = JoystickReader('/fake/js0', emit)
    with patch('os.open', return_value=42) as opened, patch('os.close') as closed, \
         patch('os.read', side_effect=[packet(kind=0x81), packet(), BlockingIOError(),
                                      OSError(errno.ENODEV, 'removed')]):
        reader.poll()
        assert reader.fd == 42
        assert opened.call_args.args[1] & __import__('os').O_NONBLOCK
        assert not opened.call_args.args[1] & __import__('os').O_RDWR
        inputs = [call.args[2] for call in emit.call_args_list if call.args[0] == 'input']
        assert [item['initial'] for item in inputs] == [True, False]
        reader.poll()
        closed.assert_called_once_with(42)
        assert reader.fd is None
        reader.retry_at = 0
        with patch('os.read', side_effect=BlockingIOError()):
            reader.poll()
        assert reader.fd == 42 and opened.call_count == 2
        reader.close()


def test_missing_device_does_not_flood_status():
    emit = Mock()
    reader = JoystickReader('/fake/js0', emit)
    with patch('os.open', side_effect=PermissionError(errno.EACCES, 'denied')):
        reader.poll()
        reader.retry_at = 0
        reader.poll()
    assert emit.call_count == 1
    assert reader.fd is None


def test_record_csv_json_roundtrip_and_restart(tmp_path):
    recorder = EventRecorder(tmp_path)
    recorder.start()
    first = recorder.path
    event = {'seq': 1, 'time': '2026-10-27T19:00:00+08:00', 'elapsed_s': 0.1,
             'source': 'input', 'kind': 'button', 'data': {'text': '按键, "test"\n', 'value': 1}}
    recorder.write(event)
    assert json.loads(first.read_text()) == event  # flushed while still open
    with first.with_suffix('.csv').open(encoding='utf-8-sig', newline='') as stream:
        row = next(csv.DictReader(stream))
    assert json.loads(row['data_json']) == event['data']
    recorder.close()
    recorder.write(event)
    assert len(first.read_text().splitlines()) == 1
    recorder.start()
    assert recorder.path != first and recorder.rows == 0
    recorder.close()


def test_recording_write_failure_disables_recording(tmp_path):
    recorder = EventRecorder(tmp_path)
    recorder.start()
    real_file = recorder.json_file
    recorder.json_file = Mock()
    recorder.json_file.write.side_effect = OSError(errno.ENOSPC, 'disk full')
    with pytest.raises(OSError):
        recorder.write({'seq': 1})
    assert not recorder.active
    real_file.close()


def test_command_preserves_unit_duration_and_multiple_ids():
    msg = SimpleNamespace(duration=0.1, position_unit='rad', position=[
        SimpleNamespace(id=6, position=0.5), SimpleNamespace(id=1, position=0.0)])
    data = servo_command_data(msg, '/servo_controller')
    assert data['unit'] == 'rad' and data['duration_s'] == 0.1
    assert data['positions'] == [{'id': 6, 'position': 0.5}, {'id': 1, 'position': 0.0}]


def test_monitor_node_only_subscribes_and_converts_real_message_types():
    from peripherals.gamepad_monitor import create_monitor_node, TOPICS
    from servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoStateList, ServoState
    from ros_robot_controller_msgs.msg import ServosPosition as DriverPositions, ServoPosition as DriverPosition
    class FakeNode:
        def __init__(self, *args, **kwargs):
            self.subscriptions = {}
        def resolve_topic_name(self, topic):
            return '/test' + topic
        def create_subscription(self, cls, topic, callback, qos):
            self.subscriptions[topic] = (cls, callback)
    emit = Mock()
    with patch('rclpy.node.Node', FakeNode):
        node = create_monitor_node(emit)
    assert set(node.subscriptions) == set(TOPICS.values())
    node.subscriptions[TOPICS['command']][1](ServosPosition(
        duration=0.1, position_unit='pulse', position=[ServoPosition(id=5, position=645.0)]))
    node.subscriptions[TOPICS['driver']][1](DriverPositions(
        duration=0.1, position=[DriverPosition(id=5, position=645)]))
    node.subscriptions[TOPICS['state']][1](ServoStateList(
        servo_state=[ServoState(id=5, position=645)]))
    assert [call.args[0] for call in emit.call_args_list] == ['command', 'driver', 'state']
    assert emit.call_args_list[1].args[2]['unit'] == 'pulse'
    assert emit.call_args_list[1].args[2]['topic'] == '/test' + TOPICS['driver']
    assert emit.call_args_list[2].args[2]['positions'] == [{'id': 5, 'position': 645}]


def test_gui_recording_pause_clear_and_cache_semantics(tmp_path):
    import tkinter as tk
    from peripherals.gamepad_monitor import MonitorWindow
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('No graphical display; remaining tests are headless')
    root.withdraw()
    app = MonitorWindow(root, '/fake/js0', tmp_path)
    try:
        app.emit('input', 'button', decode_joystick_event(packet(kind=0x81)))
        assert app.input_changes == 0
        app.emit('input', 'button', decode_joystick_event(packet()))
        app.emit('input', 'axis', decode_joystick_event(packet(value=-32768, kind=2)))
        assert app.input_changes == 2
        app.emit('command', 'positions', {'unit': 'pulse', 'duration_s': 0.1,
                 'positions': [{'id': 5, 'position': 645.0}]})
        app.emit('driver', 'positions', {'unit': 'pulse', 'duration_s': 0.1,
                 'positions': [{'id': 5, 'position': 645.0}]})
        app.emit('state', 'cache', {'positions': [{'id': 5, 'position': 645}]})
        assert [str(value) for value in app.servo_table.item('5')['values'][1:4]] == ['645 pulse', '645', '645']
        before = len(app.events.get_children())
        app.paused.set(True)
        app.emit('input', 'button', decode_joystick_event(packet(value=0)))
        assert len(app.events.get_children()) == before
        app.clear_events()
        assert len(app.events.get_children()) == 0
        app.emit('device', 'status', {'message': '断开'})
        assert app.buttons[2]['text'] == '?'
        app.toggle_recording()
        records = [json.loads(line) for line in app.recorder.path.read_text().splitlines()]
        assert records[-1]['kind'] == 'stop'
        assert any(event['source'] == 'input' and event['data']['value'] == 0 for event in records)
        assert any(event['source'] == 'state' for event in records)
        root.update_idletasks()
    finally:
        app.close()
