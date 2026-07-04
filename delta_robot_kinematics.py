"""
Delta Robot Inverse & Forward Kinematics
=========================================
Robot: 3-DOF Delta Robot for weed removal
Author: Research Team, Department of Robotics
Paper: Autonomous Weed Removal using Delta Robot on Mobile Platform

Parameters:
    e  = end-effector platform radius (mm)
    f  = base platform radius (mm)
    re = lower arm length (mm)
    rf = upper arm length (mm)
"""

import numpy as np
from typing import Tuple, Optional

# ─── Robot Parameters ─────────────────────────────────────────────────────────
E  = 50.0   # end-effector triangle side (mm)
F  = 150.0  # base triangle side (mm)
RE = 230.0  # lower arm (forearm) length (mm)
RF = 120.0  # upper arm length (mm)

# Precomputed constants
sqrt3 = np.sqrt(3)
tan60 = sqrt3
sin120 = sqrt3 / 2
cos120 = -0.5
tan30  = 1 / sqrt3
sin30  = 0.5
cos30  = sqrt3 / 2


def _ik_single(x0: float, y0: float, z0: float, joint: int) -> Optional[float]:
    """
    Compute joint angle for a single actuator.
    Returns angle in degrees, or None if unreachable.
    """
    e, f, re, rf = E, F, RE, RF

    if joint == 1:   # rotate by 0°
        x, y = x0, y0
    elif joint == 2: # rotate by 120°
        x = x0 * cos120 + y0 * sin120
        y = x0 * (-sin120) + y0 * cos120
    else:            # rotate by 240°
        x = x0 * cos120 - y0 * sin120
        y = x0 * sin120  + y0 * cos120

    # Effective target in rotated frame
    y1 = -(e / 2.0) * tan30 / 2.0 + (f / 2.0) * tan30 / 2.0 - y

    # Sphere intersection
    a = 2 * z0 * rf
    b = 2 * y1 * rf
    c = x**2 + y1**2 + z0**2 + rf**2 - re**2

    discriminant = a**2 + b**2 - c**2
    if discriminant < 0:
        return None  # Point unreachable

    t = (a - np.sqrt(discriminant)) / (b + c)  # Tangent half-angle solution
    theta = 2.0 * np.degrees(np.arctan(t))
    return theta


def inverse_kinematics(x: float, y: float, z: float) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """
    Compute all three joint angles for end-effector at (x, y, z).

    Args:
        x, y, z: Target position in mm (z is typically negative)

    Returns:
        (theta1, theta2, theta3) in degrees, or None for each if unreachable
    """
    t1 = _ik_single(x, y, z, 1)
    t2 = _ik_single(x, y, z, 2)
    t3 = _ik_single(x, y, z, 3)
    return t1, t2, t3


def forward_kinematics(theta1: float, theta2: float, theta3: float) -> Optional[Tuple[float, float, float]]:
    """
    Compute end-effector position from joint angles.

    Args:
        theta1, theta2, theta3: Joint angles in degrees

    Returns:
        (x, y, z) end-effector position in mm, or None if invalid
    """
    e, f, re, rf = E, F, RE, RF

    t1 = np.radians(theta1)
    t2 = np.radians(theta2)
    t3 = np.radians(theta3)

    # Virtual wrist positions
    y1 = -(f / 2.0) * tan30 / 2.0 + rf * np.cos(t1) - e / 2.0 * tan30 / 2.0
    z1 = -rf * np.sin(t1)

    x2 = sin60 if False else (f / 2.0) * 0
    y2 = (f / 4.0) * tan30 + rf * np.cos(t2) - e * tan30 / 4.0
    z2 = -rf * np.sin(t2)

    y3 = (f / 4.0) * tan30 + rf * np.cos(t3) - e * tan30 / 4.0
    z3 = -rf * np.sin(t3)

    # Simplified analytical solution for symmetric case
    w1 = y1**2 + z1**2
    w2 = y2**2 + z2**2
    w3 = y3**2 + z3**2

    try:
        dnm = (y2 - y1) * x2 * 2
        if abs(dnm) < 1e-6:
            return None

        a = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / (2 * dnm)
        c = -((w2 - w1) * (z3 - z1) - (w3 - w1) * (z2 - z1)) / (2 * a + 1e-9)

        # Sphere intersection
        a_coef = 1 + (b / (a + 1e-9))**2
        b_coef = 2 * (c + b * y1 / (a + 1e-9) - z1)
        c_coef = y1**2 + c**2 + z1**2 - re**2 + 2 * c * y1

        disc = b_coef**2 - 4 * a_coef * c_coef
        if disc < 0:
            return None

        z = -0.5 * (b_coef + np.sqrt(disc)) / a_coef
        x = b + c * z / (a + 1e-9)
        y = c + b * z / (a + 1e-9)
        return x, y, z
    except Exception:
        return None


def plan_weed_removal_trajectory(weed_positions: list,
                                  approach_height: float = -50.0,
                                  cut_depth: float = -170.0,
                                  speed_mm_per_s: float = 80.0) -> list:
    """
    Generate a joint-space trajectory for visiting and removing a list of weeds.

    Args:
        weed_positions: List of (x, y) tuples for each weed location (mm)
        approach_height: Z height to move above each weed before descending
        cut_depth: Z depth to cut the weed root
        speed_mm_per_s: Desired end-effector speed

    Returns:
        List of trajectory waypoints: {pos, angles, phase}
    """
    trajectory = []
    HOME = (0.0, 0.0, approach_height)

    # Go home first
    home_angles = inverse_kinematics(*HOME)
    trajectory.append({'pos': HOME, 'angles': home_angles, 'phase': 'home'})

    for wx, wy in weed_positions:
        # 1. Move above weed
        above = (wx, wy, approach_height)
        angles_above = inverse_kinematics(*above)
        if any(a is None for a in angles_above):
            continue  # Skip unreachable
        trajectory.append({'pos': above, 'angles': angles_above, 'phase': 'approach'})

        # 2. Plunge to cut depth
        cut_pos = (wx, wy, cut_depth)
        angles_cut = inverse_kinematics(*cut_pos)
        if any(a is None for a in angles_cut):
            continue
        trajectory.append({'pos': cut_pos, 'angles': angles_cut, 'phase': 'cut'})

        # 3. Retract
        trajectory.append({'pos': above, 'angles': angles_above, 'phase': 'retract'})

    # Return home
    trajectory.append({'pos': HOME, 'angles': home_angles, 'phase': 'home'})
    return trajectory


# ─── Self-test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Delta Robot Kinematics Test")
    print("=" * 40)

    # Test IK at various positions
    test_positions = [
        (0, 0, -150),
        (50, 0, -160),
        (0, 50, -155),
        (-50, 50, -140),
        (100, 0, -130),
    ]

    for pos in test_positions:
        t1, t2, t3 = inverse_kinematics(*pos)
        if all(v is not None for v in (t1, t2, t3)):
            print(f"  IK({pos[0]:6.1f}, {pos[1]:6.1f}, {pos[2]:6.1f}) → "
                  f"θ₁={t1:6.2f}° θ₂={t2:6.2f}° θ₃={t3:6.2f}°")
        else:
            print(f"  IK({pos}) → UNREACHABLE")

    # Test trajectory planning
    weeds = [(30, 20), (-40, 10), (0, -60), (80, -30)]
    traj = plan_weed_removal_trajectory(weeds)
    print(f"\nTrajectory for {len(weeds)} weeds: {len(traj)} waypoints")
    for wp in traj:
        print(f"  {wp['phase']:10s} → pos={tuple(f'{v:.1f}' for v in wp['pos'])}")
