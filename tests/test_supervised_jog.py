"""Motion state-machine tests, using a transport that never opens hardware."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/driver/kinematics'))
from kinematics.deployment import DeploymentSolver
from kinematics.supervised_jog import SupervisedJog


class FakeBus:
    def __init__(self):
        self.pulses = (500, 600, 825, 110, 537)
        self.moves, self.stops, self.reads = [], 0, 0
        self.error = None
        self.exclusive = True

    def read(self):
        self.reads += 1
        if self.error == 'timeout' and self.moves:
            raise ValueError('feedback timeout')
        if self.error == 'tracking' and self.moves:
            return tuple(p+20 for p in self.pulses)
        return self.pulses

    def prepare(self):
        pass

    def check_exclusive(self):
        if not self.exclusive:
            raise ValueError('other publisher')

    def move(self, pulses, duration):
        self.moves.append((pulses, duration))
        self.pulses = tuple(pulses)

    def wait(self, duration):
        if self.error == 'interrupt':
            raise KeyboardInterrupt()
        if self.error == 'lost_stop':
            raise RuntimeError('connection lost')

    def stop(self):
        self.stops += 1
        if self.error == 'lost_stop':
            raise RuntimeError('stop transport unavailable')


@pytest.fixture
def rig():
    bus = FakeBus()
    return SupervisedJog(DeploymentSolver(), bus), bus


def test_preview_never_moves(rig):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    assert not bus.moves and bus.stops == 0
    assert plan.start == bus.pulses
    assert max(np.abs(np.array(plan.path)-plan.start).flat) <= 20
    assert all(p[4] == plan.start[4] for p in plan.path)


def test_success_measures_every_segment_and_reports(rig):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    reports = []
    runner.execute(plan, lambda report: reports.append(report['status']))
    assert runner.report['status'] == 'completed'
    assert len(runner.report['checkpoints']) == len(plan.path)-1 == len(bus.moves)
    assert bus.reads == 2+2*len(bus.moves)
    assert runner.report['final_position_error_m'] <= .002
    assert bus.stops == 0 and reports[-1] == 'completed'
    with pytest.raises(ValueError, match='consumed'):
        runner.execute(plan)


@pytest.mark.parametrize('error,exception', [('timeout', ValueError), ('tracking', ValueError),
                                            ('interrupt', KeyboardInterrupt), ('lost_stop', RuntimeError)])
def test_failure_stops_without_sending_remaining_segments(rig, error, exception):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    bus.error = error
    with pytest.raises(exception):
        runner.execute(plan)
    assert len(bus.moves) == 1 and bus.stops == 1
    assert runner.report['status'] == 'aborted'
    assert runner.report['stop_requested']
    if error == 'lost_stop':
        assert runner.report['stop_publication'].startswith('failed:')


@pytest.mark.parametrize('case', ['drift', 'conflict', 'report'])
def test_preflight_failures_do_not_move(rig, case):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    if case == 'drift':
        bus.pulses = (501, *bus.pulses[1:])
    if case == 'conflict':
        bus.exclusive = False
    def save(report):
        if case == 'report':
            raise OSError('disk full')
    with pytest.raises((ValueError, OSError)):
        runner.execute(plan, save)
    assert not bus.moves and bus.stops == 0


@pytest.mark.parametrize('delta,duration', [([0, 0, 0], 2), ([0, 0, 6], 2),
                                           ([5, 5, 0], 2), ([float('nan'), 0, 0], 2),
                                           ([0, 0, 3], 0), ([0, 0, 3], float('inf'))])
def test_invalid_inputs_rejected_before_reading(rig, delta, duration):
    runner, bus = rig
    with pytest.raises(ValueError):
        runner.plan(delta, duration)
    assert bus.reads == 0 and not bus.moves


def test_independent_final_residual_failure_stops(rig):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    # Independent of the pulse tracking gate: require an unrealistically exact
    # final pose after planning so quantization must fail final acceptance.
    runner.solver.position_tolerance = 1e-12
    with pytest.raises(ValueError, match='Final'):
        runner.execute(plan)
    assert bus.stops == 1 and runner.report['status'] == 'aborted'


def test_report_error_after_command_still_requests_stop(rig):
    runner, bus = rig
    plan = runner.plan([0, 0, 3])
    def save(report):
        if report['executed']:
            raise OSError('disk full')
    with pytest.raises(OSError):
        runner.execute(plan, save)
    assert len(bus.moves) == 1 and bus.stops == 1
