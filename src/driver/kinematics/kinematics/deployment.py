"""Validated pulse solutions and complete trajectories; no ROS or hardware I/O."""
from dataclasses import dataclass
import math
import time

import numpy as np

from .python_kinematics import ArmKinematics, SERVO_IDS, _finite, _vector


@dataclass(frozen=True)
class PulseSolution:
    pulses: tuple
    pose: object
    requested_pitch: float
    position_error_m: float
    pitch_error_deg: float
    variation: float


class Feedback:
    """Store only complete measured snapshots, ordered by physical servo IDs."""

    def __init__(self, model, timeout=1., clock=time.monotonic):
        self.model = model
        self.timeout = _finite(timeout, 'feedback_timeout')
        if self.timeout <= 0:
            raise ValueError('feedback_timeout must be positive')
        self.clock = clock
        self.invalidate()

    def invalidate(self):
        self.pulses = None
        self.timestamp = None

    def update(self, pairs, sampled_at=None):
        self.invalidate()
        values = {}
        for servo_id, pulse in pairs:
            if servo_id not in SERVO_IDS or servo_id in values:
                raise ValueError('Expected exactly one measurement for each ID 6,5,4,3,2')
            values[servo_id] = pulse
        if set(values) != set(SERVO_IDS):
            raise ValueError('Incomplete servo measurements')
        pulses = _vector([values[i] for i in SERVO_IDS], 5, 'feedback')
        if np.any(pulses != np.rint(pulses)):
            raise ValueError('Feedback pulses must be integers')
        self.model.pulses_to_angles(pulses)
        now = self.clock()
        stamp = now if sampled_at is None else _finite(sampled_at, 'sampled_at')
        if not 0 <= now - stamp <= self.timeout:
            raise ValueError('Measurements already expired')
        self.pulses = tuple(int(p) for p in pulses)
        self.timestamp = stamp

    def get(self):
        if self.pulses is None or not 0 <= self.clock() - self.timestamp <= self.timeout:
            raise ValueError('Fresh measured positions for IDs 6,5,4,3,2 are required')
        return self.pulses


def rotation_rpy_deg(r):
    """Fixed-axis XYZ roll/pitch/yaw of the FK rotation, in degrees."""
    sy = math.hypot(r[0, 0], r[1, 0])
    if sy > 1e-9:
        angles = (math.atan2(r[2, 1], r[2, 2]), math.atan2(-r[2, 0], sy),
                  math.atan2(r[1, 0], r[0, 0]))
    else:
        angles = (math.atan2(-r[1, 2], r[1, 1]), math.atan2(-r[2, 0], sy), 0.)
    return [math.degrees(v) for v in angles]


class DeploymentSolver:
    """Preferred pitch first, then closest pitch with a representable solution.

    Limits and residuals apply AFTER integer pulse quantization. A trajectory is
    returned only if every point passes; it never skips an unreachable segment.
    This does not perform collision checking or execute servo commands.
    """

    def __init__(self, model=None, position_tolerance=0.002, pitch_tolerance=0.5,
                 max_velocity=250., max_acceleration=5000., max_samples=501):
        self.model = model if model is not None else ArmKinematics()
        for name, value in (('position_tolerance', position_tolerance),
                            ('pitch_tolerance', pitch_tolerance),
                            ('max_velocity', max_velocity),
                            ('max_acceleration', max_acceleration)):
            value = _finite(value, name)
            if value <= 0:
                raise ValueError(f'{name} must be positive')
            setattr(self, name, value)
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or not 2 <= max_samples <= 5001:
            raise ValueError('max_samples must be an integer in [2,5001]')
        self.max_samples = max_samples

    def _quantize(self, angles):
        raw = self.model.angles_to_pulses(angles)
        # Adjacent integer choices also handle limits not on an integer pulse.
        pulses = []
        scale = np.array([.24, -.24, .24, -.24, .24])
        offsets = np.array([0., -90., 0., -90., 0.])
        for i, pulse in enumerate(raw):
            choices = []
            for value in {math.floor(pulse), math.ceil(pulse)}:
                angle = math.radians((value - self.model.zero_pulses[i]) * scale[i] + offsets[i])
                if 0 <= value <= 1000 and self.model.limits[i, 0] - 1e-10 <= angle <= self.model.limits[i, 1] + 1e-10:
                    choices.append(value)
            if not choices:
                raise ValueError('No integer pulse inside joint limit')
            pulses.append(min(choices, key=lambda value: (abs(value-pulse), value)))
        return tuple(pulses)

    def _at_pitch(self, target, pitch, current):
        seed = self.model.pulses_to_angles(current)
        accepted = []
        for solution in self.model.ik(target, pitch, wrist_angle_rad=seed[4], seed=seed):
            try:
                pulses = self._quantize(solution.angles_rad)
                pose = self.model.fk(self.model.pulses_to_angles(pulses))
            except ValueError:
                continue
            error = float(np.linalg.norm(pose.position - target))
            pitch_error = abs((pose.pitch_deg - pitch + 180) % 360 - 180)
            if error <= self.position_tolerance and pitch_error <= self.pitch_tolerance:
                accepted.append(PulseSolution(pulses, pose, pitch, error, pitch_error,
                                              float(np.sum(np.abs(np.array(pulses)-current)))))
        return min(accepted, key=lambda s: (s.variation, s.position_error_m)) if accepted else None

    def solve(self, position, pitch, pitch_range, resolution, current):
        target = _vector(position, 3, 'position')
        current = _vector(current, 5, 'current_pulses')
        self.model.pulses_to_angles(current)
        pitch = _finite(pitch, 'pitch')
        bounds = _vector(pitch_range, 2, 'pitch_range')
        step = _finite(resolution, 'resolution')
        if not -180 <= bounds[0] <= pitch <= bounds[1] <= 180:
            raise ValueError('Require -180 <= pitch_range[0] <= pitch <= pitch_range[1] <= 180')
        width = bounds[1] - bounds[0]
        if step <= 0 or (width and step < width / 360):
            raise ValueError('Pitch grid limited to 361 samples; resolution must be positive')
        pitches = {pitch, float(bounds[1])}
        pitches.update(float(bounds[0] + i*step) for i in range(math.floor(width/step)+1))
        for candidate in sorted(pitches, key=lambda p: (abs(p-pitch), p)):
            result = self._at_pitch(target, candidate, current)
            if result is not None:
                return result
        raise ValueError('No reachable, representable solution within residual tolerances')

    def trajectory(self, position, pitch, pitch_range, resolution, current, duration, dt=0.02):
        duration, dt = _finite(duration, 'duration'), _finite(dt, 'dt')
        if duration <= 0 or dt <= 0 or duration / dt > self.max_samples - 1:
            raise ValueError('Invalid duration/dt or trajectory sample limit exceeded')
        steps = max(1, math.ceil(duration/dt))
        current = _vector(current, 5, 'current_pulses')
        start = self.model.fk(self.model.pulses_to_angles(current))
        end = self.solve(position, pitch, pitch_range, resolution, current)
        target = _vector(position, 3, 'position')
        delta_pitch = (end.requested_pitch - start.pitch_deg + 180) % 360 - 180
        path = [tuple(int(p) for p in current)]
        for index in range(1, steps+1):
            u = index/steps
            blend = 10*u**3 - 15*u**4 + 6*u**5
            point = start.position + blend*(target-start.position)
            angle = (start.pitch_deg + blend*delta_pitch + 180) % 360 - 180
            result = self._at_pitch(point, angle, path[-1])
            if result is None:
                raise ValueError(f'Trajectory unreachable at sample {index}/{steps}')
            path.append(result.pulses)
        # Bound finite differences of the actual integer command sequence,
        # including the start and stop. Quantization contributes acceleration.
        period = duration/steps
        velocities = np.diff(np.asarray(path, dtype=float), axis=0)/period
        if np.max(np.abs(velocities)) > self.max_velocity + 1e-9:
            raise ValueError('Trajectory exceeds pulse velocity limit; increase duration')
        accelerations = np.diff(np.vstack([np.zeros(5), velocities, np.zeros(5)]), axis=0)/period
        if np.max(np.abs(accelerations)) > self.max_acceleration + 1e-9:
            raise ValueError('Trajectory exceeds pulse acceleration limit')
        return path, end
