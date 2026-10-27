"""Hardware-free regression tests for serial selection and relocated resources."""
import os
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/driver/sdk'), str(ROOT / 'src/driver/ros_robot_controller')]
from sdk.hw_interfaces import HwInterfaces
from sdk.paths import workspace_path
from ros_robot_controller.ros_robot_controller_sdk import Board


def test_wrapper_passes_device_and_baudrate():
    with patch('sdk.hw_interfaces.Board') as board:
        HwInterfaces('/dev/test-arm', 115200)
        board.assert_called_once_with(device='/dev/test-arm', baudrate=115200)


def test_sim_does_not_open_board():
    with patch('sdk.hw_interfaces.Board') as board:
        interface = HwInterfaces(use_sim=True)
        assert interface.board is None
        board.assert_not_called()


@pytest.mark.parametrize('explicit', [False, True])
def test_board_serial_precedence(monkeypatch, explicit):
    monkeypatch.setenv('ARMPI_SERIAL_PORT', '/dev/from-env')
    monkeypatch.setenv('ARMPI_BAUDRATE', '115200')
    with patch('ros_robot_controller.ros_robot_controller_sdk.serial.Serial') as serial, \
         patch('ros_robot_controller.ros_robot_controller_sdk.threading.Thread'), \
         patch('ros_robot_controller.ros_robot_controller_sdk.time.sleep'):
        Board(**({'device': '/dev/explicit', 'baudrate': 1000000} if explicit else {}))
        serial.assert_called_once_with(None, 1000000 if explicit else 115200, timeout=5)
        serial.return_value.setPort.assert_called_once_with('/dev/explicit' if explicit else '/dev/from-env')
        serial.return_value.open.assert_called_once()


def test_board_legacy_default(monkeypatch):
    monkeypatch.delenv('ARMPI_SERIAL_PORT', raising=False)
    monkeypatch.delenv('ARMPI_BAUDRATE', raising=False)
    with patch('ros_robot_controller.ros_robot_controller_sdk.serial.Serial') as serial, \
         patch('ros_robot_controller.ros_robot_controller_sdk.threading.Thread'), \
         patch('ros_robot_controller.ros_robot_controller_sdk.time.sleep'):
        Board()
        serial.return_value.setPort.assert_called_once_with('/dev/rrc')
        serial.assert_called_once_with(None, 1000000, timeout=5)


def test_workspace_can_move_and_preserves_separator(monkeypatch, tmp_path):
    (tmp_path / 'src').mkdir()
    monkeypatch.setenv('ARMPI_WS', str(tmp_path))
    assert workspace_path('src/app/config/') == str(tmp_path / 'src/app/config') + '/'


def test_source_workspace_auto_discovery(monkeypatch):
    monkeypatch.delenv('ARMPI_WS', raising=False)
    assert workspace_path('src/app/config/lab_config.yaml') == str(ROOT / 'src/app/config/lab_config.yaml')


def test_invalid_workspace_is_explicit(monkeypatch, tmp_path):
    monkeypatch.setenv('ARMPI_WS', str(tmp_path / 'missing'))
    with pytest.raises(RuntimeError, match='ARMPI_WS'):
        workspace_path('src/app/config/')


def test_core_launch_conditions_and_includes(monkeypatch):
    import runpy
    from launch import LaunchContext
    monkeypatch.setenv('CHASSIS_TYPE', 'None')
    context = LaunchContext()
    context.launch_configurations['enable_kinematics'] = 'false'
    module = runpy.run_path(str(ROOT / 'src/driver/sdk/launch/armpi_ultra.launch.py'))
    actions = module['launch_setup'](context)
    assert len(actions) == 3
    assert not actions[2].condition.evaluate(context)
    assert dict(actions[1].launch_arguments)['enable_grasp'].perform(context) == 'false'
    for action in actions:
        assert action.launch_description_source.get_launch_description(context) is not None


def test_launch_parameters_reach_ros_node(monkeypatch):
    import runpy
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    context = LaunchContext()
    context.launch_configurations.update(serial_port='/dev/test:arm', baudrate='115200')
    with patch('launch_ros.actions.Node') as node:
        runpy.run_path(str(ROOT / 'src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py'))['generate_launch_description']()
    parameters = evaluate_parameters(context, normalize_parameters(node.call_args.kwargs['parameters']))
    assert list(parameters) == [{'serial_port': '/dev/test:arm', 'baudrate': 115200}]


def test_usb_camera_device_override():
    import runpy
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    context = LaunchContext()
    context.launch_configurations['video_device'] = '/dev/v4l/by-id/test-camera'
    with patch('launch_ros.actions.Node') as node:
        runpy.run_path(str(ROOT / 'src/peripherals/launch/include/usb_cam.launch.py'))['generate_launch_description']()
    parameters = evaluate_parameters(context, normalize_parameters(node.call_args_list[0].kwargs['parameters']))
    assert parameters[1]['video_device'] == '/dev/v4l/by-id/test-camera'
