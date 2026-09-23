#!/usr/bin/env python3
"""Zero-pose inter-robot collision check for the dual Fairino workbench.

Pure forward kinematics (no ROS graph needed): loads the model parameter
YAMLs from fr_description, places the two robots on the workbench exactly
like dual_fr_cell.launch.py does, and computes the worst-case clearance
between the two kinematic chains (each link approximated by a capsule).

Demonstrates why the ros2_control initial positions matter:

  - at the zero configuration (all joints 0.0) the arms stretch horizontally
    across the table and each flange ends up inside the other robot's base
    volume -> the two robots would collide;
  - at the configured initial pose (shoulder -90 deg, elbow +90 deg) the arms
    fold up and the chains are well separated -> safe.

Usage: python3 zero_pose_collision_check.py [model]
"""

import math
import os
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODELS = os.path.join(REPO, 'fr_description', 'config')

# Workbench layout, mirrors launch/dual_fr_cell.launch.py
TABLE_TOP_Z = 0.75
ALICE = {'pos': np.array([-0.4, 0.0, TABLE_TOP_Z]), 'yaw': math.pi}
BOB = {'pos': np.array([0.4, 0.0, TABLE_TOP_Z]), 'yaw': 0.0}

# Capsule radius per segment (joint-origin chain, see fk_points)
SEG_RADIUS = [0.10,   # base column
              0.08,   # upper arm
              0.07,   # forearm
              0.06,   # wrist 1
              0.06,   # wrist 2
              0.05]   # wrist 3 to flange

ZERO = [0.0] * 6
CLASSIC_READY = [0.0, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]      # standalone initial_positions.yaml
WORKBENCH = [math.pi / 2, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]  # workbench initial pose


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def fk_points(model, q, placement):
    """Joint origins + flange of one arm in world coordinates at configuration q.

    Segment endpoints are joint origins (the shoulder_pan origin is degenerate
    at the base, so the base column runs directly to the shoulder_lift origin).
    """
    kin = yaml.safe_load(open(os.path.join(MODELS, model, 'default_kinematics.yaml')))
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    T = np.eye(4)
    T[:3, :3] = rotz(placement['yaw'])
    T[:3, 3] = placement['pos']
    frames = [placement['pos'].copy()]           # base
    for jname, qj in zip(joints, q):
        k = kin['kinematics'][jname]
        Tj = np.eye(4)
        Tj[:3, :3] = rpy_mat(*[float(v) for v in k['rpy'].split()])
        Tj[:3, 3] = [float(v) for v in k['xyz'].split()]
        Rj = np.eye(4)
        Rj[:3, :3] = rotz(qj)
        T = T @ Tj @ Rj
        frames.append(T[:3, 3].copy())            # joint origins
    flange = frames[-1] + (T[:3, :3] @ np.array([0.0, 0.0, 1.0])) * kin['flange_offset']
    # 7 points: base, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, flange
    # (the shoulder_pan origin collapses onto the base point)
    return [frames[0]] + frames[2:] + [flange]


SEG_NAMES = ['base column', 'upper arm', 'forearm', 'wrist 1', 'wrist 2',
             'wrist 3 to flange']


def seg_point(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


def seg_seg(a1, b1, a2, b2):
    best = float('inf')
    for t in np.linspace(0.0, 1.0, 41):
        best = min(best, seg_point(a1 + t * (b1 - a1), a2, b2))
    return best


def worst_clearance(model, q_a, q_b):
    """Returns (min clearance in m, description of the worst link pair)."""
    fa = fk_points(model, q_a, ALICE)
    fb = fk_points(model, q_b, BOB)
    worst = float('inf')
    worst_desc = None
    for i in range(len(SEG_RADIUS)):
        for j in range(len(SEG_RADIUS)):
            d = seg_seg(fa[i], fa[i + 1], fb[j], fb[j + 1])
            r = SEG_RADIUS[i] + SEG_RADIUS[j]
            if d - r < worst:
                worst = d - r
                worst_desc = (f'{SEG_NAMES[i]}(alice) vs {SEG_NAMES[j]}(bob): '
                              f'link distance {d:.4f} m, radii sum {r:.2f} m')
    return worst, worst_desc


def main():
    model = sys.argv[1] if len(sys.argv) > 1 else 'fairino5_v6'
    print(f'model: {model}   layout: alice (-0.4 m, 180 deg) / bob (+0.4 m, 0 deg)')

    ok = True
    for name, q, expect in [
        ('zero configuration (all joints 0)', ZERO, 'collision'),
        ('classic ready pose [0, -90, +90 deg]', CLASSIC_READY, 'collision'),
        ('workbench pose [pan +90, -90, +90 deg]', WORKBENCH, 'clear'),
    ]:
        d, desc = worst_clearance(model, q, q)
        collides = d < 0
        good = (collides and expect == 'collision') or (not collides and expect == 'clear')
        ok = ok and good
        print(f'\n{name}: clearance {d:+.4f} m   [{desc}]')
        if collides:
            print('  -> COLLISION' + (' (expected)' if expect == 'collision' else ' (UNEXPECTED!)'))
        else:
            print('  -> clear' + (' (expected)' if expect == 'clear' else ' (UNEXPECTED!)'))
    print('\nLesson: the ros2_control initial_value configuration is part of the '
          'scene,\nnot of the robot. On this workbench every robot must start folded '
          'up and turned\ntoward its own table end (dual_fr_cell.launch.py passes '
          'workbench_initial_positions.yaml).')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
