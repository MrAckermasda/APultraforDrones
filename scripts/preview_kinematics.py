#!/usr/bin/env python3
"""Offline Python FK/IK preview. Does not import ROS or connect to hardware."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/driver/kinematics'))
from kinematics.python_kinematics import ArmGeometry, ArmKinematics, SERVO_IDS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--pulses', nargs=5, type=float, metavar=('ID6','ID5','ID4','ID3','ID2'))
    source.add_argument('--angles-deg', nargs=5, type=float)
    parser.add_argument('--chassis', default=os.environ.get('CHASSIS_TYPE', 'None'),
                        choices=('None', 'Mecanum', 'Tank', 'Slide_Rails'))
    parser.add_argument('--target', nargs=3, type=float, help='XYZ in metres; requires --pitch')
    parser.add_argument('--pitch', type=float, help='Geometric tool pitch in degrees, not Euler pitch')
    parser.add_argument('--wrist-deg', type=float, help='q5; defaults to seed q5')
    parser.add_argument('--json', type=Path, help='Save the numeric report')
    parser.add_argument('--plot', type=Path, help='Save a static PNG; no GUI is opened')
    args = parser.parse_args()
    if args.target is not None and args.pitch is None:
        parser.error('--target requires --pitch')
    model = ArmKinematics(ArmGeometry.for_chassis(args.chassis))
    try:
        if args.angles_deg is not None:
            seed = [math.radians(x) for x in args.angles_deg]
        else:
            seed = model.pulses_to_angles(args.pulses if args.pulses is not None else [500,600,825,110,500])
        pose = model.fk(seed)
        target = pose.position.tolist() if args.target is None else args.target
        pitch = pose.pitch_deg if args.pitch is None else args.pitch
        wrist = seed[4] if args.wrist_deg is None else math.radians(args.wrist_deg)
        solutions = model.ik(target, pitch, wrist, seed=seed)
        report = {
            'implementation': 'experimental Python Modified DH + geometric IK',
            'hardware_verified': False,
            'assumptions': ['tool offset along final +Z', 'pitch measured in q1 radial plane',
                            'no collision/trajectory checks; no vendor equivalence claim'],
            'chassis': args.chassis, 'geometry_m': asdict(model.geometry),
            'servo_ids': SERVO_IDS,
            'input_angles_deg': [math.degrees(x) for x in seed],
            'fk': {'position_m': pose.position.tolist(), 'quaternion_xyzw': pose.quaternion_xyzw.tolist(),
                   'rotation': pose.rotation.tolist(), 'geometric_pitch_deg': pose.pitch_deg},
            'ik_request': {'position_m': target, 'pitch_deg': pitch, 'wrist_angle_deg': math.degrees(wrist)},
            'solution_count': len(solutions),
            'solutions': [{**asdict(s), 'angles_deg': [math.degrees(x) for x in s.angles_rad],
                           'float_pulses': model.angles_to_pulses(s.angles_rad).tolist()} for s in solutions],
        }
    except ValueError as exc:
        parser.error(str(exc))
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    print(text)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(text + '\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection='3d')
        points = pose.joint_points
        ax.plot(*points.T, 'o-', label='FK input pose', color='#2166ac', linewidth=3)
        clouds = [points, np.array(target).reshape(1,3)]
        if solutions:
            solved = model.fk(solutions[0].angles_rad).joint_points
            ax.plot(*solved.T, 'o--', label='IK solution nearest seed', color='#d95f02', linewidth=2)
            clouds.append(solved)
        ax.scatter(*target, marker='x', s=100, color='#13854b', label='Requested target')
        all_points = np.concatenate(clouds)
        center = (all_points.min(axis=0) + all_points.max(axis=0))/2
        half = max(float(np.ptp(all_points, axis=0).max())/2 * 1.2, 0.1)
        ax.set_xlim(center[0]-half, center[0]+half)
        ax.set_ylim(center[1]-half, center[1]+half)
        ax.set_zlim(center[2]-half, center[2]+half)
        ax.set_box_aspect((1,1,1))
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title('ArmPi Ultra: offline FK / IK preview\nModel assumptions NOT verified on hardware')
        ax.legend(loc='upper left')
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=160, bbox_inches='tight')
        plt.close(fig)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
