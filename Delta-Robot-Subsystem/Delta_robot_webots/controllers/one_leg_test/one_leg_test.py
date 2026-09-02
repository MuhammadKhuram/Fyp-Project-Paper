"""
One-arm Webots test controller for the delta robot leg.
Kinematic-driven, no physics: computes IK, writes pose directly to
Solid fields. Snap-to-pose (see webots-delta-robot-spec.md) - no
interpolation.
"""
import math
import numpy as np
from controller import Supervisor
from delta_robot_kinematics import DeltaKinematics

TIME_STEP = 32           # ms - arbitrary, motion is snap-to-pose
HOLD_TIME_S = 2.0        # how long to hold each test pose

WORLD_OFFSET_MM = np.array([0.0, 0.0, 300.0])


def outward(phi_deg):
    """Unit vector pointing radially outward for a leg at phi_deg"""
    p = math.radians(phi_deg)
    return np.array([math.sin(p), -math.cos(p), 0.0])


def orient_solid(node, point_a_mm, point_b_mm):
    """Point a rod-shaped Solid from point_a to point_b."""
    a_m, b_m = np.array(point_a_mm) / 1000.0, np.array(point_b_mm) / 1000.0
    direction = b_m - a_m
    length = np.linalg.norm(direction)
    if length < 1e-9:
        return
    direction = direction / length

    # Webots ENU coordinate system aligns Cylinder primitives with the Z-axis
    default_axis = np.array([0.0, 0.0, 1.0])
    dot = np.clip(np.dot(default_axis, direction), -1.0, 1.0)
    axis = np.cross(default_axis, direction)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 1e-9:
        rotation = [1.0, 0.0, 0.0, 0.0] if dot > 0 else [1.0, 0.0, 0.0, math.pi]
    else:
        axis = axis / axis_norm
        angle = math.acos(dot)
        rotation = [axis[0], axis[1], axis[2], angle]

    node.getField("translation").setSFVec3f(((a_m + b_m) / 2.0).tolist())
    node.getField("rotation").setSFRotation(rotation)


class OneLegRig:
    LEG_PHI_DEG = 0.0

    def __init__(self, supervisor):
        self.kin = DeltaKinematics()
        self.upper_arm = supervisor.getFromDef("UPPER_ARM")
        self.forearm_a = supervisor.getFromDef("FOREARM_A")
        self.forearm_b = supervisor.getFromDef("FOREARM_B")
        self.target_marker = supervisor.getFromDef("TARGET")

        self.base_joint = self.kin.wb * outward(self.LEG_PHI_DEG) 

    def move_to(self, x0, y0, z0):
        theta1, _, _ = self.kin.inverse(x0, y0, z0, enforce_limits=True, check_collision=False)
        if theta1 is None:
            print(f"Target ({x0}, {y0}, {z0}) unreachable for leg 1 - skipping")
            return False

        u = outward(self.LEG_PHI_DEG)
        raw_wrist = self.kin._wrist_point(theta1, self.LEG_PHI_DEG)
        true_wrist = raw_wrist + self.kin.wp * u
        platform_pt = np.array([x0, y0, z0]) + self.kin.wp * u

        # Calculate a vector perpendicular to the leg's swinging plane
        # to separate forearm_a and forearm_b into a parallelogram 
        lateral_vec = np.cross(np.array([0.0, 0.0, 1.0]), u) * 15.0  # 15mm offset

        # Draw Upper Arm
        orient_solid(self.upper_arm, self.base_joint + WORLD_OFFSET_MM, true_wrist + WORLD_OFFSET_MM)
        
        # Draw Forearm Parallelogram (shifted left and right)
        orient_solid(self.forearm_a, true_wrist + WORLD_OFFSET_MM + lateral_vec, platform_pt + WORLD_OFFSET_MM + lateral_vec)
        orient_solid(self.forearm_b, true_wrist + WORLD_OFFSET_MM - lateral_vec, platform_pt + WORLD_OFFSET_MM - lateral_vec)

        if self.target_marker is not None:
            self.target_marker.getField("translation").setSFVec3f(
                ((np.array([x0, y0, z0]) + WORLD_OFFSET_MM) / 1000.0).tolist()
            )
        return True


def main():
    supervisor = Supervisor()
    rig = OneLegRig(supervisor)

    test_points = [
        (0, 0, -180),
        (50, 0, -160),
        (0, 50, -200),
        (-50, -50, -170),
    ]

    idx = 0
    last_switch = 0.0
    while supervisor.step(TIME_STEP) != -1:
        t = supervisor.getTime()
        if t - last_switch > HOLD_TIME_S:
            rig.move_to(*test_points[idx % len(test_points)])
            idx += 1
            last_switch = t


if __name__ == "__main__":
    main()
