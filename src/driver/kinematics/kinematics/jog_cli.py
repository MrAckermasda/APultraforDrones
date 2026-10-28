"""Preview or execute a <=5 mm Cartesian jog on a validated ArmPi model."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from ros_robot_controller_msgs.msg import (GetBusServoCmd, ServoPosition, ServosPosition,
                                           BusServoState, SetBusServoState)
from ros_robot_controller_msgs.srv import GetBusServoState

from .deployment import DeploymentSolver, Feedback
from .python_kinematics import ArmGeometry, ArmKinematics, NOMINAL_LIMITS_DEG, SERVO_IDS
from .supervised_jog import SupervisedJog


def load_solver(path):
    settings = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    defaults = ArmGeometry.for_chassis(os.environ.get('CHASSIS_TYPE', 'None'))
    geometry = ArmGeometry(**{key: settings.get(key, getattr(defaults, key))
                              for key in defaults.__dataclass_fields__})
    model = ArmKinematics(geometry,
                          np.asarray(settings.get('joint_limits_deg', NOMINAL_LIMITS_DEG)).reshape(5, 2),
                          settings.get('zero_pulses', [500.]*5))
    solver = DeploymentSolver(model, **{key: settings[key] for key in (
        'position_tolerance', 'pitch_tolerance', 'max_velocity', 'max_acceleration', 'max_samples')
        if key in settings})
    return solver


class RosTransport(Node):
    def __init__(self, model, namespace, timeout=1.):
        super().__init__('kinematics_jog_' + str(os.getpid()))
        if not math.isfinite(timeout) or not .1 <= timeout <= 5:
            raise ValueError('Read timeout must be in [0.1,5] seconds')
        self.timeout = timeout
        self.feedback = Feedback(model, timeout)
        prefix = namespace.rstrip('/') + '/bus_servo/'
        self.position_topic, self.state_topic = prefix+'set_position', prefix+'set_state'
        self.reader = self.create_client(GetBusServoState, prefix+'get_state')
        self.motion = self.stop_publisher = None

    def read(self):
        self.feedback.invalidate()
        if not self.reader.wait_for_service(timeout_sec=self.timeout):
            raise ValueError('Measured bus feedback service unavailable')
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            command = GetBusServoCmd()
            command.id, command.get_id, command.get_position = servo_id, 1, 1
            request.cmd.append(command)
        started = time.monotonic()
        future = self.reader.call_async(request)
        try:
            while not future.done() and time.monotonic()-started < self.timeout:
                if not rclpy.ok():
                    raise RuntimeError('ROS shutdown during feedback read')
                rclpy.spin_once(self, timeout_sec=.01)
            if not future.done():
                raise ValueError('Measured feedback timed out; no automatic retry')
            response = future.result()
            if response is None or not response.success or len(response.state) != 5:
                raise ValueError('Measured feedback failed or incomplete')
            pairs = []
            for state in response.state:
                if len(state.present_id) != 1 or len(state.position) != 1:
                    raise ValueError('Malformed measured servo position')
                pairs.append((state.present_id[0], state.position[0]))
            self.feedback.update(pairs, started)
            return self.feedback.get()
        finally:
            if not future.done():
                self.reader.remove_pending_request(future)
                future.cancel()

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError('ROS shutdown during movement')
            rclpy.spin_once(self, timeout_sec=min(.01, max(0., deadline-time.monotonic())))

    def prepare(self):
        self.motion = self.create_publisher(ServosPosition, self.position_topic, 1)
        self.stop_publisher = self.create_publisher(SetBusServoState, self.state_topic, 1)
        deadline = time.monotonic()+2.
        while not (self.motion.get_subscription_count() and self.stop_publisher.get_subscription_count()):
            if time.monotonic() > deadline:
                raise ValueError('Motion/stop subscribers not discovered')
            self.wait(.02)
        self.wait(.2)

    def check_exclusive(self):
        for topic in (self.position_topic, self.state_topic):
            if self.count_subscribers(topic) != 1:
                raise ValueError(f'Expected exactly one controller subscriber on {topic}')
            others = [info for info in self.get_publishers_info_by_topic(topic)
                      if (info.node_name, info.node_namespace) != (self.get_name(), self.get_namespace())]
            if others or self.count_publishers(topic) != 1:
                raise ValueError(f'Other command publisher on {topic}; stop controller manager/gamepad/actions')

    def move(self, pulses, duration):
        message = ServosPosition()
        message.duration = float(duration)
        for servo_id, pulse in zip(SERVO_IDS, pulses):
            joint = ServoPosition()
            joint.id, joint.position = servo_id, int(pulse)
            message.position.append(joint)
        self.motion.publish(message)

    def stop(self):
        message = SetBusServoState()
        for servo_id in SERVO_IDS:
            state = BusServoState()
            state.present_id, state.stop = [1, servo_id], [1]
            message.state.append(state)
        # Ignore a second Ctrl+C while sending the best-effort stop request.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            self.stop_publisher.publish(message)
            self.wait(.15)
        finally:
            signal.signal(signal.SIGINT, previous)


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.jog-', delete=False) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delta-mm', nargs=3, type=float, required=True, metavar=('DX', 'DY', 'DZ'))
    parser.add_argument('--duration', type=float, default=2., help='Sum of segment motion durations; excludes reads/settling')
    parser.add_argument('--config', type=Path, help='Same calibrated YAML used by python_kinematics')
    parser.add_argument('--controller', default='/ros_robot_controller', help='Controller node namespace')
    parser.add_argument('--read-timeout', type=float, default=1.)
    parser.add_argument('--execute', action='store_true', help='Send motion; default only reads/plans')
    parser.add_argument('--report', type=Path, help='JSON report path (default log/kinematics_jog/<timestamp>-<pid>.json)')
    options = parser.parse_args(args)
    config = options.config or Path(get_package_share_directory('kinematics')) / 'config/python_kinematics.yaml'
    report_path = options.report or Path('log/kinematics_jog') / (time.strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}.json')
    node = runner = None
    lock = None
    code = 1
    try:
        # Same-user local lock only. ROS publisher checks additionally detect
        # other visible command writers, but cannot enforce a distributed lock.
        lock = open(Path(tempfile.gettempdir()) / f'armpi-kinematics-jog-{os.getuid()}.lock', 'a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        solver = load_solver(config)
        rclpy.init(args=[])
        # Keep ROS alive on Ctrl+C so the exception handler can publish stop.
        from rclpy.signals import uninstall_signal_handlers
        uninstall_signal_handlers()
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
        node = RosTransport(solver.model, options.controller, options.read_timeout)
        runner = SupervisedJog(solver, node)
        runner.report.update(config=str(config.resolve()), controller=options.controller,
                             delta_mm=options.delta_mm, execute_requested=options.execute,
                             validation_scope='Joint feedback and model FK, not independent TCP measurement',
                             model_geometry=vars(solver.model.geometry),
                             zero_pulses=solver.model.zero_pulses.tolist(),
                             joint_limits_deg=np.degrees(solver.model.limits).tolist())
        plan = runner.plan(options.delta_mm, options.duration)
        write_report(report_path, runner.report)
        print(f'Start pulses (IDs 6,5,4,3,2): {list(plan.start)}', flush=True)
        print(f'Target XYZ (m): {list(plan.target_m)}; end pulses: {list(plan.path[-1])}', flush=True)
        print(f'{len(plan.path)-1} measured segments; motion time {options.duration}s plus reads/settling', flush=True)
        if options.execute:
            runner.execute(plan, lambda report: write_report(report_path, report))
            print('Completed; see measured checkpoints in report.', flush=True)
        else:
            print('Preview only: no motion publisher created. --execute enables movement.', flush=True)
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Jog aborted: {exc or type(exc).__name__}', flush=True)
        if runner is not None:
            runner.report.update(status='aborted', error=str(exc) or type(exc).__name__)
            try:
                write_report(report_path, runner.report)
            except (OSError, ValueError, TypeError) as report_error:
                print(f'Report write failed: {report_error}', flush=True)
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if lock is not None:
                lock.close()
        finally:
            signal.signal(signal.SIGINT, previous)
        print(f'Report path: {report_path}', flush=True)
    return code
