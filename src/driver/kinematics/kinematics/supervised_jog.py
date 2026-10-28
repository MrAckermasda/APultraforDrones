"""Small Cartesian moves with measured checkpoints, independent of ROS I/O."""
from dataclasses import dataclass
import time

import numpy as np

from .python_kinematics import _finite, _vector


@dataclass(frozen=True)
class JogPlan:
    start: tuple
    target_m: tuple
    pitch_deg: float
    path: tuple
    segment_duration: float


class SupervisedJog:
    """Move, settle, measure each segment. This is not a real-time controller.

    transport supplies read(), prepare(), check_exclusive(), move(), wait(), stop().
    Only execute() writes motion. Exceptions after a possible write request a stop
    for the five arm joints; stop delivery cannot be guaranteed by this layer.
    """

    def __init__(self, solver, transport, tracking_tolerance=5., settle_time=.1):
        self.solver, self.transport = solver, transport
        self.tracking_tolerance = _finite(tracking_tolerance, 'tracking_tolerance')
        self.settle_time = _finite(settle_time, 'settle_time')
        if not 0 <= self.tracking_tolerance <= 10 or not 0 <= self.settle_time <= 1:
            raise ValueError('Require tracking tolerance 0..10 pulses and settling time 0..1 s')
        self._plan = None
        self.report = {'status': 'not_started', 'executed': False, 'checkpoints': []}

    def _pulses(self, values):
        p = _vector(values, 5, 'measured pulses')
        if np.any(p != np.rint(p)):
            raise ValueError('Measured pulses must be integers')
        self.solver.model.pulses_to_angles(p)
        return tuple(int(v) for v in p)

    def plan(self, delta_mm, duration=2.):
        delta = _vector(delta_mm, 3, 'delta_mm') / 1000.
        duration = _finite(duration, 'duration')
        if not 0 < np.linalg.norm(delta) <= .005 + 1e-12:
            raise ValueError('Commissioning jog requires a nonzero displacement of at most 5 mm')
        if not 1 <= duration <= 10:
            raise ValueError('Commissioning duration must be between 1 and 10 seconds')
        current = self._pulses(self.transport.read())
        start = self.solver.model.fk(self.solver.model.pulses_to_angles(current))
        target = start.position + delta
        path, _ = self.solver.trajectory(target, start.pitch_deg,
                                          [start.pitch_deg, start.pitch_deg], 1.,
                                          current, duration, dt=.2)
        pulses = np.asarray(path)
        if np.max(np.abs(pulses-np.array(current))) > 20:
            raise ValueError('Jog exceeds 20 pulse excursion; choose a smaller displacement')
        if np.max(np.abs(np.diff(pulses, axis=0))) > 5:
            raise ValueError('Jog exceeds 5 pulses per segment; increase duration or reduce displacement')
        if np.any(pulses[:, 4] != current[4]):
            raise ValueError('Wrist rotation must remain fixed')
        if np.array_equal(pulses[-1], current):
            raise ValueError('Target is below pulse resolution; no movement would be produced')
        plan = JogPlan(current, tuple(float(x) for x in target), float(start.pitch_deg),
                       tuple(tuple(int(v) for v in p) for p in path), duration/(len(path)-1))
        self.report.update(status='planned', start_pulses=list(current),
                           start_position_m=start.position.tolist(), target_position_m=list(plan.target_m),
                           geometric_pitch_deg=plan.pitch_deg, path=[list(p) for p in plan.path],
                           segment_duration_s=plan.segment_duration,
                           timing='Stop and measure between segments; elapsed time exceeds requested duration')
        self._plan = plan
        return plan

    def execute(self, plan, save=lambda report: None):
        # Plans can only be consumed by the runner which produced them; the CLI
        # never loads executable paths from JSON reports.
        if self.report['status'] != 'planned' or plan is not self._plan:
            raise ValueError('Plan does not match this runner or was already consumed')
        started = time.monotonic()
        may_have_moved = False
        try:
            self.transport.prepare()
            self.transport.check_exclusive()
            current = self._pulses(self.transport.read())
            if current != plan.start:
                raise ValueError('Start position changed since planning; rerun preview')
            self.report['status'] = 'executing'
            save(self.report)  # Fail before moving if the report cannot be written.
            for index, target in enumerate(plan.path[1:], 1):
                self.transport.check_exclusive()
                # Measure again before each segment to detect external movement
                # during the report write or other inter-segment work.
                current = self._pulses(self.transport.read())
                previous = np.array(plan.path[index-1])
                if np.max(np.abs(np.array(current)-previous)) > self.tracking_tolerance:
                    raise ValueError('Measured position drifted before next segment')
                jump = float(np.max(np.abs(np.array(target)-current)))
                if jump > 10 or jump / plan.segment_duration > self.solver.max_velocity:
                    raise ValueError('Measured-to-target jump exceeds pulse/velocity bound')
                may_have_moved = True
                self.report['executed'] = True
                self.transport.move(target, plan.segment_duration)
                self.transport.wait(plan.segment_duration + self.settle_time)
                actual = self._pulses(self.transport.read())
                pulse_error = float(np.max(np.abs(np.array(actual)-target)))
                pose = self.solver.model.fk(self.solver.model.pulses_to_angles(actual))
                checkpoint = {'index': index, 'target_pulses': list(target),
                              'measured_pulses': list(actual), 'max_pulse_error': pulse_error,
                              'fk_from_measured_pulses_m': pose.position.tolist()}
                self.report['checkpoints'].append(checkpoint)
                save(self.report)
                if pulse_error > self.tracking_tolerance:
                    raise ValueError(f'Segment {index} tracking error {pulse_error} pulses exceeds tolerance')
            position_error = float(np.linalg.norm(pose.position-np.array(plan.target_m)))
            pitch_error = abs((pose.pitch_deg-plan.pitch_deg+180) % 360-180)
            self.report.update(final_position_error_m=position_error, final_pitch_error_deg=pitch_error)
            if position_error > self.solver.position_tolerance or pitch_error > self.solver.pitch_tolerance:
                raise ValueError('Final measured-joint FK residual exceeds configured tolerance')
            self.report['status'] = 'completed'
        except BaseException as exc:
            self.report.update(status='aborted', error=str(exc) or type(exc).__name__)
            if may_have_moved:
                self.report['stop_requested'] = True
                try:
                    self.transport.stop()
                    self.report['stop_publication'] = 'sent; physical stop is not acknowledged'
                except BaseException as stop_error:
                    self.report['stop_publication'] = f'failed: {stop_error}'
            raise
        finally:
            self.report['elapsed_s'] = time.monotonic()-started
            save(self.report)
