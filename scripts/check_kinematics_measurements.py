#!/usr/bin/env python3
"""Compare measured TCP positions to FK. Reads files only; never opens serial."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/driver/kinematics'))
from kinematics.python_kinematics import ArmGeometry, ArmKinematics, NOMINAL_LIMITS_DEG, _vector


def check_measurements(samples, model, tolerance):
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('tolerance must be finite and positive')
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError('Supply at least three independently measured poses')
    results = []
    seen = set()
    for index, sample in enumerate(samples):
        pulses = _vector(sample['pulses'], 5, 'pulses')
        measured = _vector(sample['position_m'], 3, 'position_m')
        seen.add(tuple(pulses))
        pose = model.fk(model.pulses_to_angles(pulses))
        error = float(np.linalg.norm(pose.position-measured))
        results.append({'sample': index, 'predicted_position_m': pose.position.tolist(),
                        'position_error_m': error, 'passed': error <= tolerance})
    if len(seen) < 3:
        raise ValueError('At least three distinct joint poses are required')
    return {'passed': all(r['passed'] for r in results),
            'max_position_error_m': max(r['position_error_m'] for r in results),
            'samples': results,
            'scope': 'Position comparison only; orientation, collision and motion are not validated'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements', type=Path, help='JSON array: pulses in ID 6..2 order, position_m in arm base frame')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] /
                        'src/driver/kinematics/config/python_kinematics.yaml')
    parser.add_argument('--chassis', choices=['None', 'Mecanum', 'Tank', 'Slide_Rails'], default='None')
    parser.add_argument('--tolerance-m', type=float, default=.005)
    args = parser.parse_args()
    try:
        settings = yaml.safe_load(args.config.read_text())['/**']['ros__parameters']
        defaults = ArmGeometry.for_chassis(args.chassis)
        geometry = ArmGeometry(**{name: settings.get(name, getattr(defaults, name))
                                  for name in defaults.__dataclass_fields__})
        limits = np.asarray(settings.get('joint_limits_deg', NOMINAL_LIMITS_DEG)).reshape(5, 2)
        model = ArmKinematics(geometry, limits, settings.get('zero_pulses', [500]*5))
        report = check_measurements(json.loads(args.measurements.read_text()), model, args.tolerance_m)
    except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
        parser.exit(2, f'Measurement check failed: {exc}\n')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
