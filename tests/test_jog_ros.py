"""ROS transport and installed CLI against a simulated bus, never real hardware."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import threading
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('ros_robot_controller_msgs.srv')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, ServosPosition, SetBusServoState
from ros_robot_controller_msgs.srv import GetBusServoState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/driver/kinematics'))
from kinematics.python_kinematics import ArmKinematics
from kinematics.jog_cli import RosTransport, load_solver


@pytest.fixture
def bus():
    rclpy.init()
    node = Node('fake_jog_bus')
    data = {'pulses': [500, 600, 825, 110, 537], 'moves': [], 'stops': [], 'reads': 0,
            'success': True, 'hold_position': False}
    def read(request, response):
        assert [cmd.id for cmd in request.cmd] == [6, 5, 4, 3, 2]
        assert all(cmd.get_id and cmd.get_position for cmd in request.cmd)
        data['reads'] += 1
        response.success = data['success']
        for servo_id, pulse in list(zip([6, 5, 4, 3, 2], data['pulses']))[::-1]:
            state = BusServoState()
            state.present_id, state.position = [servo_id], [pulse]
            response.state.append(state)
        return response
    def move(message):
        assert [joint.id for joint in message.position] == [6, 5, 4, 3, 2]
        data['moves'].append(message)
        if not data['hold_position']:
            data['pulses'] = [joint.position for joint in message.position]
    node.create_service(GetBusServoState, '/fake_jog_bus/bus_servo/get_state', read)
    node.create_subscription(ServosPosition, '/fake_jog_bus/bus_servo/set_position', move, 10)
    node.create_subscription(SetBusServoState, '/fake_jog_bus/bus_servo/set_state', data['stops'].append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node, data
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


def test_transport_read_preview_and_failed_snapshot(bus):
    _, data = bus
    transport = RosTransport(ArmKinematics(), '/fake_jog_bus')
    try:
        assert transport.read() == tuple(data['pulses'])
        assert transport.motion is None and transport.stop_publisher is None
        data['success'] = False
        with pytest.raises(ValueError, match='failed'):
            transport.read()
        assert transport.feedback.pulses is None
    finally:
        transport.destroy_node()
    assert not data['moves'] and not data['stops']


def test_transport_conflict_and_stop_message(bus):
    node, data = bus
    transport = RosTransport(ArmKinematics(), '/fake_jog_bus')
    other = node.create_publisher(ServosPosition, '/fake_jog_bus/bus_servo/set_position', 1)
    try:
        transport.prepare()
        transport.wait(.2)
        with pytest.raises(ValueError, match='Other command publisher'):
            transport.check_exclusive()
        node.destroy_publisher(other)
        transport.wait(.2)
        transport.check_exclusive()
        transport.stop()
        assert data['stops']
        assert [list(s.present_id) for s in data['stops'][-1].state] == [[1, i] for i in (6, 5, 4, 3, 2)]
        assert all(list(s.stop) == [1] and not s.enable_torque for s in data['stops'][-1].state)
    finally:
        transport.destroy_node()


def test_transport_timeout_discards_pending_request(bus):
    node, data = bus
    pending = node.create_service(GetBusServoState, '/slow_jog_bus/bus_servo/get_state',
                                 lambda request, response: (time.sleep(.25), response)[1])
    transport = RosTransport(ArmKinematics(), '/slow_jog_bus', timeout=.1)
    try:
        with pytest.raises(ValueError, match='timed out'):
            transport.read()
        assert transport.feedback.pulses is None
        assert list(transport.reader._pending_requests) == []
    finally:
        transport.destroy_node()
    assert not data['moves']


def cli_command(report, execute=False):
    command = [sys.executable, '-c', 'from kinematics.jog_cli import main; raise SystemExit(main())',
               '--delta-mm', '0', '0', '3', '--duration', '1',
               '--controller', '/fake_jog_bus', '--report', str(report)]
    if execute:
        command.append('--execute')
    return command


def test_cli_preview_then_execution_with_measured_report(bus, tmp_path):
    _, data = bus
    preview, report = tmp_path/'preview.json', tmp_path/'execution.json'
    result = subprocess.run(cli_command(preview), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(preview.read_text())['status'] == 'planned'
    assert not data['moves'] and not data['stops']
    result = subprocess.run(cli_command(report, True), capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    record = json.loads(report.read_text())
    assert record['status'] == 'completed'
    assert len(record['checkpoints']) == len(data['moves']) == 5
    assert record['final_position_error_m'] <= .002
    assert not data['stops']


@pytest.mark.parametrize('interrupt', [signal.SIGINT, signal.SIGTERM])
def test_cli_interrupt_after_first_move_requests_stop(bus, tmp_path, interrupt):
    _, data = bus
    report = tmp_path/'interrupted.json'
    process = subprocess.Popen(cli_command(report, True), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic()+10
        while not data['moves'] and process.poll() is None and time.monotonic() < deadline:
            time.sleep(.01)
        assert data['moves'], 'No simulated command arrived'
        process.send_signal(interrupt)
        output, _ = process.communicate(timeout=5)
        assert process.returncode != 0, output
        assert len(data['moves']) == 1
        assert data['stops'], output
        record = json.loads(report.read_text())
        assert record['status'] == 'aborted' and record['stop_requested']
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)


def test_loader_keeps_calibration_from_yaml(tmp_path):
    path = tmp_path/'model.yaml'
    path.write_text('''/**:
  ros__parameters:
    base_height: 0.123
    zero_pulses: [501., 502., 503., 504., 505.]
    position_tolerance: 0.001
''')
    solver = load_solver(path)
    assert solver.model.geometry.base_height == .123
    assert solver.model.zero_pulses.tolist() == [501., 502., 503., 504., 505.]
    assert solver.position_tolerance == .001
