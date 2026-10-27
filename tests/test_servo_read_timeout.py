"""No hardware: bounded serial reply waits and ROS failure reporting."""
import queue
import struct
import threading
import time
from unittest.mock import Mock

import pytest
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller.ros_robot_controller_node import RosRobotController
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd


def make_board():
    board = Board.__new__(Board)
    board.servo_read_timeout = 0.03
    board.servo_read_lock = threading.Lock()
    board.bus_servo_queue = queue.Queue()
    board.buf_write = Mock()
    return board


def test_no_response_times_out_and_releases_lock():
    board = make_board()
    start = time.monotonic()
    assert board.bus_servo_read_position(1) is None
    assert time.monotonic() - start < 0.5
    assert not board.servo_read_lock.locked()
    board.buf_write.assert_called_once()


def test_busy_reader_does_not_wait_forever_or_write():
    board = make_board()
    board.servo_read_lock.acquire()
    try:
        assert board.bus_servo_read_position(1) is None
        board.buf_write.assert_not_called()
    finally:
        board.servo_read_lock.release()


def test_stale_malformed_wrong_servo_and_wrong_command_are_ignored():
    board = make_board()
    board.bus_servo_queue.put(struct.pack('<BBbh', 1, 5, 0, 999))
    def reply(*args):
        for packet in [b'bad', struct.pack('<BBbh', 2, 5, 0, 10),
                       struct.pack('<BBbh', 1, 9, 0, 20),
                       struct.pack('<BBbh', 1, 5, 0, 510)]:
            board.bus_servo_queue.put(packet)
    board.buf_write.side_effect = reply
    assert board.bus_servo_read_position(1) == [510]


def test_failed_firmware_status_returns_none():
    board = make_board()
    board.buf_write.side_effect = lambda *args: board.bus_servo_queue.put(struct.pack('<BBbh', 1, 5, -1, 0))
    assert board.bus_servo_read_position(1) is None


def test_broadcast_id_response_is_accepted():
    board = make_board()
    board.buf_write.side_effect = lambda *args: board.bus_servo_queue.put(struct.pack('<BBbB', 6, 0x12, 0, 6))
    assert board.bus_servo_read_id() == [6]


@pytest.mark.parametrize('position,expected', [(None, False), ([500], True)])
def test_service_reports_missing_data_and_preserves_response_shape(position, expected):
    node = Mock()
    node.board.bus_servo_read_id.return_value = [1]
    node.board.bus_servo_read_position.return_value = position
    request = GetBusServoState.Request(cmd=[GetBusServoCmd(id=1, get_id=1, get_position=1)])
    result = RosRobotController.get_bus_servo_state(node, request, GetBusServoState.Response())
    assert result.success == expected
    assert list(result.state[0].present_id) == [1]
    assert list(result.state[0].position) == (position or [])


def test_serial_exception_becomes_failure_response():
    node = Mock()
    node.board.bus_servo_read_id.side_effect = OSError('disconnected')
    request = GetBusServoState.Request(cmd=[GetBusServoCmd(id=1, get_id=1)])
    result = RosRobotController.get_bus_servo_state(node, request, GetBusServoState.Response())
    assert not result.success
    assert len(result.state) == 1
    node.get_logger.return_value.warning.assert_called_once()
