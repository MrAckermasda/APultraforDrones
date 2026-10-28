"""ArmPi Ultra FK/IK from transform.py's Modified DH table.

This module is explicitly separate from the vendor .so modules. It neither
imports ROS nor publishes commands. Tool +Z and pitch conventions are hypotheses
to verify against the vendor runtime/physical arm before enabling pose targets in the optional ROS backend.
Lengths: metres; joints: radians; geometric tool pitch: degrees.
"""
from dataclasses import dataclass
from itertools import product
import math

import numpy as np

TAU = 2 * math.pi
# Nominal tutorial limits; deliberately omit transform.py's 0.2 degree padding.
NOMINAL_LIMITS_DEG = ((-120, 120), (-180, 0), (-120, 120), (-200, 20), (-120, 120))
SERVO_IDS = (6, 5, 4, 3, 2)


def _vector(value, length, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain {length} finite numbers') from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {length} finite numbers')
    return result


def _finite(value, name):
    if isinstance(value, (bool, str)):
        raise ValueError(f'{name} must be a finite number')
    try:
        value = float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def _wrap(value):
    return (value + math.pi) % TAU - math.pi


@dataclass(frozen=True)
class ArmGeometry:
    base_height: float = 0.094605
    upper_arm: float = 0.10048
    forearm: float = 0.100
    wrist_length: float = 0.055
    gripper_length: float = 0.115

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = _finite(getattr(self, name), name)
            if value < 0 or (name in ('upper_arm', 'forearm') and value == 0):
                raise ValueError(f'{name} must be positive (base/tool offsets may be zero)')
            object.__setattr__(self, name, value)

    @property
    def tool_length(self):
        return self.wrist_length + self.gripper_length

    @classmethod
    def for_chassis(cls, chassis='None'):
        if chassis not in ('None', 'Mecanum', 'Tank', 'Slide_Rails'):
            raise ValueError('Unknown chassis; specify None/Mecanum/Tank/Slide_Rails')
        return cls(base_height=0.162 if chassis == 'Slide_Rails' else 0.094605)


@dataclass(frozen=True)
class FKResult:
    position: np.ndarray
    rotation: np.ndarray
    quaternion_xyzw: np.ndarray
    transform: np.ndarray
    joint_points: np.ndarray
    pitch_deg: float


@dataclass(frozen=True)
class IKSolution:
    angles_rad: tuple
    pitch_deg: float
    position_error_m: float
    pitch_error_deg: float
    base_axis_singular: bool


def modified_dh(alpha, a, theta, d=0.0):
    """Craig MDH: Rx(alpha) Tx(a) Rz(theta) Tz(d)."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0., a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0., 0., 0., 1.]])


def rotation_to_quaternion(rotation):
    """Return normalized (x,y,z,w), including stable half-turn cases."""
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)) or not np.allclose(
            r.T @ r, np.eye(3), atol=1e-9, rtol=0) or not math.isclose(
            np.linalg.det(r), 1., abs_tol=1e-9):
        raise ValueError('Expected a proper rotation matrix')
    # Symmetric eigenproblem, avoids division by trace near 180 degrees.
    k = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3.
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


class ArmKinematics:
    """Nominal 5-axis model, no collision checking or hardware execution.

    q1..q5 correspond to servo IDs 6,5,4,3,2. ID 1 is gripper opening.
    IK constrains XYZ + geometric tool pitch + chosen q5, not arbitrary 6D pose.
    """
    def __init__(self, geometry=None, joint_limits_deg=NOMINAL_LIMITS_DEG,
                 zero_pulses=(500., 500., 500., 500., 500.)):
        self.geometry = geometry if geometry is not None else ArmGeometry()
        self.zero_pulses = _vector(zero_pulses, 5, "zero_pulses").copy()
        if np.any(self.zero_pulses < 0) or np.any(self.zero_pulses > 1000):
            raise ValueError("zero_pulses must be within [0,1000]")
        self.zero_pulses.setflags(write=False)
        limits = np.asarray(joint_limits_deg, dtype=float)
        if limits.shape != (5, 2) or not np.all(np.isfinite(limits)) or np.any(
                limits[:, 0] > limits[:, 1]) or np.any(limits[:, 1] - limits[:, 0] > 360):
            raise ValueError('joint_limits_deg must be five finite ordered intervals of width <=360')
        self.limits = np.radians(limits)
        self.limits.setflags(write=False)

    def _joints(self, angles):
        q = _vector(angles, 5, 'angles_rad')
        if np.any(q < self.limits[:, 0] - 1e-10) or np.any(q > self.limits[:, 1] + 1e-10):
            raise ValueError('Joint angles outside configured limits')
        return q

    def pulses_to_angles(self, pulses):
        p = _vector(pulses, 5, 'pulses in ID order 6,5,4,3,2')
        if np.any(p < 0) or np.any(p > 1000):
            raise ValueError('Pulse values must be in [0,1000]')
        degrees = (p - self.zero_pulses) * np.array([0.24, -0.24, 0.24, -0.24, 0.24])
        degrees[[1, 3]] -= 90
        return self._joints(np.radians(degrees))

    def angles_to_pulses(self, angles):
        """Float pulses only; no rounding/clipping or command publishing."""
        degrees = np.degrees(self._joints(angles)).copy()
        degrees[[1, 3]] += 90
        p = degrees / np.array([0.24, -0.24, 0.24, -0.24, 0.24]) + self.zero_pulses
        if np.any(p < -1e-8) or np.any(p > 1000 + 1e-8):
            raise ValueError('Solution cannot be represented within [0,1000] pulses')
        return p

    def fk(self, angles_rad):
        q = self._joints(angles_rad)
        g = self.geometry
        t = np.eye(4)
        t[2, 3] = g.base_height
        points = [np.zeros(3), t[:3, 3].copy()]
        for alpha, a, theta in zip((0., -math.pi/2, 0., 0., -math.pi/2),
                                   (0., 0., g.upper_arm, g.forearm, 0.), q):
            t = t @ modified_dh(alpha, a, theta)
            points.append(t[:3, 3].copy())
        tool = np.eye(4)
        tool[2, 3] = g.tool_length  # Explicit assumption: offset along final +Z.
        t = t @ tool
        points.append(t[:3, 3].copy())
        pitch = _wrap(-q[1] - q[2] - q[3] - math.pi/2)
        return FKResult(t[:3, 3].copy(), t[:3, :3].copy(),
                        rotation_to_quaternion(t[:3, :3]), t,
                        np.array(points), math.degrees(pitch))

    def _equivalents(self, angle, index):
        lower, upper = self.limits[index]
        first = math.ceil((lower - angle - 1e-10) / TAU)
        last = math.floor((upper - angle + 1e-10) / TAU)
        return [angle + TAU * n for n in range(first, last + 1)]

    def ik(self, position, pitch_deg, wrist_angle_rad=0., seed=None):
        """All discrete elbow/yaw branches for this pitch and q5; [] if unreachable.

        pitch=0 points radially outward, +90 up, -90 down. For the negative
        radial branch, outward refers to q1's own radial direction. At x=y=0,
        choose seed q1 (otherwise 0) as one representative of a free base axis.
        seed only orders solutions; it does not change joint limits or clamp.
        """
        target = _vector(position, 3, 'position')
        pitch_deg = _finite(pitch_deg, 'pitch_deg')
        if not -180 <= pitch_deg <= 180:
            raise ValueError('pitch_deg must be in [-180,180]')
        wrist = _finite(wrist_angle_rad, 'wrist_angle_rad')
        if not self.limits[4, 0] <= wrist <= self.limits[4, 1]:
            raise ValueError('wrist_angle_rad outside joint 5 limits')
        reference = None if seed is None else self._joints(seed)
        g = self.geometry
        # Reject targets outside the enclosing sphere before squaring numbers.
        if math.hypot(float(target[0]), float(target[1]), float(target[2])-g.base_height) > (
                g.upper_arm + g.forearm + g.tool_length + 1e-8):
            return []
        rho = math.hypot(target[0], target[1])
        singular = rho < 1e-10
        if singular:
            yaw = float(reference[0]) if reference is not None else float(
                np.clip(0., self.limits[0,0], self.limits[0,1]))
            branches = [(yaw, 0.)]
        else:
            yaw = math.atan2(target[1], target[0])
            branches = [(yaw, rho), (yaw + math.pi, -rho)]
        alpha = math.radians(pitch_deg)
        solutions = []
        for yaw, radial in branches:
            u = radial - g.tool_length * math.cos(alpha)
            v = -(target[2] - g.base_height - g.tool_length * math.sin(alpha))
            cosine = (u*u + v*v - g.upper_arm**2 - g.forearm**2) / (2*g.upper_arm*g.forearm)
            if not -1 - 1e-10 <= cosine <= 1 + 1e-10:
                continue
            elbow = math.acos(max(-1., min(1., cosine)))
            for q3 in (elbow, -elbow):
                q2 = math.atan2(v, u) - math.atan2(g.forearm * math.sin(q3),
                                                 g.upper_arm + g.forearm * math.cos(q3))
                q4 = -alpha - math.pi/2 - q2 - q3
                for candidates in product(*(self._equivalents(a, i)
                                             for i, a in enumerate((yaw, q2, q3, q4)))):
                    q = tuple(candidates) + (wrist,)
                    pose = self.fk(q)
                    error = float(np.linalg.norm(pose.position - target))
                    pitch_error = abs(math.degrees(_wrap(math.radians(pose.pitch_deg - pitch_deg))))
                    if error > 1e-8 or pitch_error > 1e-7:
                        continue
                    if any(np.max(np.abs(np.array(s.angles_rad) - q)) < 1e-8 for s in solutions):
                        continue
                    solutions.append(IKSolution(q, pitch_deg, error, pitch_error, singular))
        if reference is not None:
            # Actual limited-joint travel, not modulo-2pi distance across a stop.
            solutions.sort(key=lambda s: float(np.sum(np.abs(np.array(s.angles_rad) - reference))))
        return solutions

    def ik_pitch_range(self, position, preferred_pitch, pitch_range, resolution=1.,
                       wrist_angle_rad=0., seed=None):
        """Bounded pitch grid, preferred angle first. All solutions keep their pitch.

        This is explicitly a grid search, not a proof that no solution exists
        between samples. It does not imitate undocumented vendor get_ik ordering.
        """
        bounds = _vector(pitch_range, 2, 'pitch_range')
        preferred = _finite(preferred_pitch, 'preferred_pitch')
        step = _finite(resolution, 'resolution')
        if not -180 <= bounds[0] <= bounds[1] <= 180 or not bounds[0] <= preferred <= bounds[1]:
            raise ValueError('Require -180 <= lower <= preferred <= upper <= 180')
        if step <= 0:
            raise ValueError('resolution must be positive')
        width = bounds[1] - bounds[0]
        if width and step < width / 3600:
            raise ValueError('Pitch search limited to at most 3601 grid samples')
        count = math.floor(width / step) if width else 0
        pitches = sorted(set([float(bounds[0] + i * step) for i in range(count + 1)]
                             + [float(bounds[1]), preferred]), key=lambda p: (abs(p-preferred), p))
        return [s for pitch in pitches for s in self.ik(position, pitch, wrist_angle_rad, seed)]
