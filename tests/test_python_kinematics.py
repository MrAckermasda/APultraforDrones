"""Offline geometry tests: no ROS node, driver, pygame, or serial device."""
import ast
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/driver/kinematics'))
from kinematics.python_kinematics import (ArmGeometry, ArmKinematics, modified_dh,
                                         rotation_to_quaternion)


def test_source_parameter_and_pulse_mapping_agreement():
    source = ast.parse((ROOT / 'src/driver/kinematics/kinematics/transform.py').read_text())
    literals = {}
    for statement in source.body:
        if isinstance(statement, ast.Assign) and isinstance(statement.targets[0], ast.Name):
            try:
                literals[statement.targets[0].id] = ast.literal_eval(statement.value)
            except (ValueError, TypeError):
                pass
    model = ArmKinematics()
    g = model.geometry
    assert (g.upper_arm,g.forearm,g.wrist_length,g.gripper_length) == tuple(
        literals[name] for name in ('link1','link2','link3','tool_link'))
    pulses = [100, 600, 825, 110, 900]
    expected = []
    for index, pulse in enumerate(pulses, 1):
        low, high, mid, a_low, a_high, a_mid = literals[f'joint{index}_map']
        expected.append(math.radians((pulse-mid)/(high-low)*(a_high-a_low)+a_mid))
    np.testing.assert_allclose(model.pulses_to_angles(pulses), expected)
    np.testing.assert_allclose(model.angles_to_pulses(expected), pulses)


def test_mdh_matches_independently_composed_elementary_transforms():
    alpha, a, theta, d = -0.4, 0.1, 0.7, 0.03
    rx = np.eye(4)
    rx[1:3,1:3] = [[math.cos(alpha),-math.sin(alpha)],[math.sin(alpha),math.cos(alpha)]]
    tx = np.eye(4); tx[0,3] = a
    rz = np.eye(4)
    rz[:2,:2] = [[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]
    tz = np.eye(4); tz[2,3] = d
    np.testing.assert_allclose(modified_dh(alpha,a,theta,d), rx @ tx @ rz @ tz)


def test_analytic_baseline_poses():
    model = ArmKinematics()
    g = model.geometry
    down = model.fk([0,0,0,0,0])
    np.testing.assert_allclose(down.position, [g.upper_arm+g.forearm, 0, g.base_height-g.tool_length], atol=1e-15)
    np.testing.assert_allclose(down.rotation, np.diag([1,-1,-1]), atol=1e-15)
    upright = model.fk([0,-math.pi/2,0,-math.pi/2,0])
    np.testing.assert_allclose(upright.position, [0,0,g.base_height+g.upper_arm+g.forearm+g.tool_length], atol=1e-15)
    np.testing.assert_allclose(upright.rotation, np.diag([-1,-1,1]), atol=1e-15)
    assert upright.pitch_deg == pytest.approx(90)


def test_random_fk_against_independent_planar_expansion_and_ik_full_pose():
    model = ArmKinematics()
    g = model.geometry
    rng = np.random.default_rng(20261027)
    for _ in range(400):
        q = rng.uniform(model.limits[:,0], model.limits[:,1])
        pose = model.fk(q)
        a,b,c,d,e = q
        radial = g.upper_arm*math.cos(b)+g.forearm*math.cos(b+c)-g.tool_length*math.sin(b+c+d)
        height = g.base_height-g.upper_arm*math.sin(b)-g.forearm*math.sin(b+c)-g.tool_length*math.cos(b+c+d)
        np.testing.assert_allclose(pose.position, [radial*math.cos(a),radial*math.sin(a),height], atol=1e-12)
        np.testing.assert_allclose(pose.rotation.T @ pose.rotation, np.eye(3), atol=1e-12)
        assert np.linalg.det(pose.rotation) == pytest.approx(1)
        sols = model.ik(pose.position,pose.pitch_deg,wrist_angle_rad=q[4],seed=q)
        assert sols, (q, pose.position, pose.pitch_deg)
        best = model.fk(sols[0].angles_rad)
        np.testing.assert_allclose(best.position,pose.position,atol=1e-8)
        np.testing.assert_allclose(best.rotation,pose.rotation,atol=1e-8)
        np.testing.assert_allclose(sols[0].angles_rad,q,atol=1e-8)


@pytest.mark.parametrize('axis', [0,1,2])
def test_quaternion_half_turns(axis):
    r = -np.eye(3); r[axis,axis] = 1
    q = rotation_to_quaternion(r)
    assert abs(q[axis]) == pytest.approx(1)
    assert np.linalg.norm(q) == pytest.approx(1)
    assert abs(q[3]) < 1e-12


def test_quaternion_reconstructs_rotation():
    q = ArmKinematics().fk(np.radians([37,-100,45,-80,20]))
    x,y,z,w = q.quaternion_xyzw
    matrix = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    np.testing.assert_allclose(matrix,q.rotation,atol=1e-12)


def test_both_elbows_and_base_axis_singularity():
    model = ArmKinematics()
    q = np.radians([0,-90,50,-90,0])
    pose = model.fk(q)
    sols = model.ik(pose.position,pose.pitch_deg)
    assert any(s.angles_rad[2] > 0 for s in sols)
    assert any(s.angles_rad[2] < 0 for s in sols)
    upright = np.radians([35,-90,0,-90,0])
    pose = model.fk(upright)
    sols = model.ik(pose.position,90,seed=upright)
    assert sols and all(s.base_axis_singular for s in sols)
    assert sols[0].angles_rad[0] == pytest.approx(upright[0])


def test_custom_base_height_and_tool_length():
    q = np.radians([0,-90,0,-90,0])
    normal = ArmKinematics().fk(q)
    rails = ArmKinematics(ArmGeometry.for_chassis('Slide_Rails')).fk(q)
    assert rails.position[2]-normal.position[2] == pytest.approx(0.162-0.094605)
    model = ArmKinematics(ArmGeometry(gripper_length=0.12))
    pose = model.fk(q)
    assert pose.position[2]-normal.position[2] == pytest.approx(0.005)
    assert model.ik(pose.position,pose.pitch_deg,seed=q)


def test_unreachable_and_joint_limits_do_not_return_clipped_angles():
    model = ArmKinematics()
    assert model.ik([10,0,10],0) == []
    assert model.ik([0,0,model.geometry.base_height],0) == []
    with pytest.raises(ValueError):
        model.fk(np.radians([121,-90,0,-90,0]))
    with pytest.raises(ValueError):
        model.pulses_to_angles([500,1000,500,500,500])  # pulse valid, shoulder invalid


@pytest.mark.parametrize('args', [([float('nan'),0,0],0), ([0,0],0), ([0,0,0],181)])
def test_invalid_ik_inputs(args):
    with pytest.raises(ValueError):
        ArmKinematics().ik(*args)


def test_pitch_search_bounded_and_preferred_first():
    model = ArmKinematics()
    q = np.radians([0,-100,40,-40,0])
    pose = model.fk(q)
    sols = model.ik_pitch_range(pose.position,pose.pitch_deg,[-90,90],10,seed=q)
    assert sols and sols[0].pitch_deg == pytest.approx(pose.pitch_deg)
    for step in (0,-1,1e-300,float('nan')):
        with pytest.raises(ValueError):
            model.ik_pitch_range(pose.position,0,[-180,180],step)


def test_python_module_has_no_ros_or_hardware_imports():
    program = "import sys;sys.path.insert(0,'src/driver/kinematics');from kinematics.python_kinematics import ArmKinematics;ArmKinematics().fk([0,0,0,0,0]);assert not any(x in sys.modules for x in ('rclpy','serial','pygame','geometry_msgs'))"
    subprocess.run([sys.executable,'-c',program],cwd=ROOT,check=True)


@pytest.mark.parametrize('index', range(5))
def test_nominal_joint_boundary_roundtrips(index):
    model = ArmKinematics()
    for limit in model.limits[index]:
        q = np.radians([0,-100,40,-80,10])
        q[index] = limit
        pose = model.fk(q)
        solutions = model.ik(pose.position,pose.pitch_deg,q[4],q)
        assert solutions
        np.testing.assert_allclose(model.fk(solutions[0].angles_rad).rotation,pose.rotation,atol=1e-8)
        np.testing.assert_allclose(model.fk(solutions[0].angles_rad).position,pose.position,atol=1e-8)


def test_wrist_rotation_preserves_tool_position():
    model = ArmKinematics()
    q = np.radians([20,-80,-20,-70,0])
    first = model.fk(q)
    q[4] = 1.0
    second = model.fk(q)
    np.testing.assert_allclose(first.position,second.position,atol=1e-12)
    assert not np.allclose(first.rotation,second.rotation)


def test_invalid_models_rotations_and_large_unreachable_target():
    with pytest.raises(ValueError):
        ArmGeometry(upper_arm=0)
    with pytest.raises(ValueError):
        ArmGeometry(base_height=float('inf'))
    with pytest.raises(ValueError):
        ArmKinematics(joint_limits_deg=[[-180,180]]*4)
    with pytest.raises(ValueError):
        rotation_to_quaternion(np.diag([1,1,-1]))
    assert ArmKinematics().ik([1e300,0,0],0) == []
