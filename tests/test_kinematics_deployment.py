"""Deployment safety and integer-output tests; no serial device is opened."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/driver/kinematics'))
from kinematics.python_kinematics import ArmKinematics, SERVO_IDS
from kinematics.deployment import DeploymentSolver, Feedback

PULSES = (500, 600, 825, 110, 537)


def test_feedback_reorders_ids_and_expires():
    clock = [10.]
    feedback = Feedback(ArmKinematics(), clock=lambda: clock[0])
    with pytest.raises(ValueError):
        feedback.get()
    feedback.update(list(zip(SERVO_IDS, PULSES))[::-1], 9.9)
    assert feedback.get() == PULSES
    clock[0] = 11.
    with pytest.raises(ValueError):
        feedback.get()


@pytest.mark.parametrize('pairs', [list(zip(SERVO_IDS, PULSES))[:-1],
                                  [(6, 500)] * 5,
                                  list(zip(SERVO_IDS, [500, 1000, 500, 500, 500])),
                                  list(zip(SERVO_IDS, [500.5, 600, 825, 110, 537]))])
def test_bad_feedback_invalidates_previous_snapshot(pairs):
    feedback = Feedback(ArmKinematics())
    feedback.update(zip(SERVO_IDS, PULSES))
    with pytest.raises(ValueError):
        feedback.update(pairs)
    with pytest.raises(ValueError):
        feedback.get()


def test_delayed_measurement_rejected():
    feedback = Feedback(ArmKinematics(), clock=lambda: 10.)
    with pytest.raises(ValueError):
        feedback.update(zip(SERVO_IDS, PULSES), 8.)


def test_calibration_offsets_roundtrip_and_pulse_bounds():
    model = ArmKinematics(zero_pulses=[502, 493, 505, 498, 501])
    q = model.pulses_to_angles(PULSES)
    np.testing.assert_allclose(model.angles_to_pulses(q), PULSES)
    with pytest.raises(ValueError):
        model.angles_to_pulses(np.radians([120, -90, 0, -90, 0]))
    with pytest.raises(ValueError):
        ArmKinematics(zero_pulses=[500]*4)


def test_integer_solution_preserves_wrist_and_rechecks_fk():
    solver = DeploymentSolver()
    pose = solver.model.fk(solver.model.pulses_to_angles(PULSES))
    result = solver.solve(pose.position, pose.pitch_deg, [-90, 90], 1., PULSES)
    assert result.pulses == PULSES
    assert result.variation == 0.
    target = [0.20, 0., 0.20]
    result = solver.solve(target, 0., [-90, 90], 1., PULSES)
    assert result.pulses[4] == PULSES[4]
    assert all(isinstance(p, int) for p in result.pulses)
    assert np.linalg.norm(result.pose.position-target) <= solver.position_tolerance
    assert result.pitch_error_deg <= solver.pitch_tolerance


def test_rounding_stays_inside_noninteger_joint_boundary():
    model = ArmKinematics()
    solver = DeploymentSolver(model)
    q = np.radians([0, -180, 0, -200, 0])
    pulses = solver._quantize(q)
    angles = model.pulses_to_angles(pulses)
    assert np.all(angles >= model.limits[:, 0]-1e-10)
    assert np.all(angles <= model.limits[:, 1]+1e-10)


@pytest.mark.parametrize('target,pitch,bounds,step', [
    ([10., 0., 0.], 0., [-90, 90], 1.),
    ([float('nan'), 0., 0.], 0., [-90, 90], 1.),
    ([.2, 0., .2], 0., [-90, 90], 0.),
    ([.2, 0., .2], 0., [-90, 90], 1e-20),
    ([.2, 0., .2], 0., [10, -10], 1.),
])
def test_invalid_or_unreachable_requests(target, pitch, bounds, step):
    with pytest.raises(ValueError):
        DeploymentSolver().solve(target, pitch, bounds, step, PULSES)


def test_trajectory_endpoints_zero_distance_and_velocity():
    solver = DeploymentSolver()
    pose = solver.model.fk(solver.model.pulses_to_angles(PULSES))
    path, _ = solver.trajectory(pose.position, pose.pitch_deg, [-90, 90], 1., PULSES, .2)
    assert path == [PULSES] * 11
    target = pose.position + [.002, 0, .002]
    path, _ = solver.trajectory(target, pose.pitch_deg, [-90, 90], 1., PULSES, 1.)
    assert len(path) == 51
    assert path[0] == PULSES
    end = solver.model.fk(solver.model.pulses_to_angles(path[-1]))
    assert np.linalg.norm(end.position-target) <= .002
    assert max(abs(np.diff(np.array(path), axis=0)).flatten()) / .02 <= solver.max_velocity
    solver.max_velocity = .1
    with pytest.raises(ValueError, match='velocity'):
        solver.trajectory(target, pose.pitch_deg, [-90, 90], 1., PULSES, 1.)


@pytest.mark.parametrize('duration', [0, -1, float('nan'), float('inf'), 1e9])
def test_trajectory_rejects_invalid_duration(duration):
    with pytest.raises(ValueError):
        DeploymentSolver().trajectory([.2, 0, .2], 0., [-90, 90], 1., PULSES, duration)


def test_trajectory_aborts_instead_of_returning_partial_path(monkeypatch):
    solver = DeploymentSolver()
    pose = solver.model.fk(solver.model.pulses_to_angles(PULSES))
    original = solver._at_pitch
    count = [0]
    def unreachable_middle(*args):
        count[0] += 1
        return None if count[0] == 4 else original(*args)
    monkeypatch.setattr(solver, '_at_pitch', unreachable_middle)
    with pytest.raises(ValueError, match='sample'):
        solver.trajectory(pose.position, pose.pitch_deg, [-90, 90], 1., PULSES, 1.)


def test_measurement_checker_detects_independent_fixture_offset():
    import importlib.util
    path = Path(__file__).resolve().parents[1] / 'scripts/check_kinematics_measurements.py'
    spec = importlib.util.spec_from_file_location('measurement_check', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = ArmKinematics()
    # Synthetic inputs exercise the checker only; these are NOT hardware evidence.
    samples = []
    for base in (400, 500, 600):
        pulses = [base, 600, 825, 110, 500]
        position = model.fk(model.pulses_to_angles(pulses)).position.tolist()
        samples.append({'pulses': pulses, 'position_m': position})
    assert module.check_measurements(samples, model, .005)['passed']
    samples[1]['position_m'][2] += .01
    assert not module.check_measurements(samples, model, .005)['passed']
    with pytest.raises(ValueError):
        module.check_measurements([samples[0]]*3, model, .005)
