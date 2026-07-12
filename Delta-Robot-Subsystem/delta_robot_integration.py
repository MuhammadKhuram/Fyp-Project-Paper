import csv
import numpy as np

import config as cfg
from delta_robot_kinematics import DeltaKinematics

# Everything about how the delta arm talks to subsystems that don't exist
# yet - the camera and the mobile base navigation. Imports DeltaKinematics
# instead of redefining it. All numbers pulled from config.py.


class CameraInterface:
    """Placeholder camera-frame -> robot-frame transform. Pure translation
    for now, no rotation, because there's no real camera mounted or
    calibrated yet.

    Assumes the camera looks straight down, axes parallel to the delta's
    own frame, offset by a fixed vector. If the real camera ends up
    tilted, this needs a rotation matrix, not just an offset - don't
    forget that when the hardware shows up.
    """

    def __init__(self, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        self.offset = np.array([offset_x, offset_y, offset_z])

    def to_robot_frame(self, x_cam, y_cam, z_cam):
        return tuple(np.array([x_cam, y_cam, z_cam]) + self.offset)

    def batch_to_robot_frame(self, detections):
        return [self.to_robot_frame(*d) for d in detections]


class BaseCoupling:
    """We're assuming the mobile base stops moving while the arm clears
    everything in the current camera frame, then drives to the next
    patch. Simplest thing that works given no navigation subsystem
    exists yet - say so plainly in the paper, don't let it read like
    continuous-motion cutting is handled, because it isn't.
    """

    def __init__(self, kin: DeltaKinematics, operating_radius=None):
        self.kin = kin
        self.operating_radius = operating_radius or cfg.OPERATING_RADIUS

    def check_reachable(self, x, y, z):
        return all(a is not None for a in self.kin.inverse(x, y, z))

    def suggest_reposition(self, x, y, z):
        """Unreachable point -> how far (dx, dy) the base should shift to
        bring it back inside reach. None if it's already reachable."""
        if self.check_reachable(x, y, z):
            return None
        r = np.hypot(x, y)
        if r == 0:
            return (0.0, 0.0)
        target_r = self.operating_radius * 0.9
        scale = target_r / r
        return (x * (1 - scale), y * (1 - scale))


def estimate_leg_travel_time(angles_a, angles_b):
    deltas = [abs(a - b) for a, b in zip(angles_a, angles_b) if a is not None and b is not None]
    return max(deltas) / cfg.JOINT_SPEED_DEG_S if deltas else 0.0


def estimate_cycle_time(kin: DeltaKinematics, trajectory_xyz, home_angles=(0.0, 0.0, 0.0)):
    total = 0.0
    prev_angles = home_angles
    for pt in trajectory_xyz:
        angles = kin.inverse(*pt)
        if any(a is None for a in angles):
            continue
        total += estimate_leg_travel_time(prev_angles, angles)
        total += cfg.SETTLE_TIME_S
        total += cfg.CUT_DWELL_S
        prev_angles = angles
    total += estimate_leg_travel_time(prev_angles, home_angles)
    total += cfg.SETTLE_TIME_S
    return total


def run_pipeline(kin, cam, coupling, camera_detections, z_cut=-170.0):
    """camera detections -> robot frame -> reachability check -> greedy
    ordering -> cycle time. Returns a dict, doesn't print anything, so it
    can be reused by the sensitivity analysis without spamming stdout."""
    robot_pts = cam.batch_to_robot_frame(camera_detections)
    reachable, unreachable = [], []
    for rob_pt in robot_pts:
        if coupling.check_reachable(*rob_pt):
            reachable.append(rob_pt)
        else:
            unreachable.append((rob_pt, coupling.suggest_reposition(*rob_pt)))

    traj = []
    if reachable:
        xy = [(p[0], p[1]) for p in reachable]
        curr, order, unvisited = [0.0, 0.0], [], list(xy)
        while unvisited:
            nxt = min(unvisited, key=lambda w: ((w[0]-curr[0])**2 + (w[1]-curr[1])**2)**0.5)
            order.append(nxt); unvisited.remove(nxt); curr = nxt
        traj = [(x, y, z_cut) for x, y in order]

    cycle_time = estimate_cycle_time(kin, traj) if traj else 0.0

    return {
        "n_detected": len(camera_detections),
        "n_reachable": len(reachable),
        "n_unreachable": len(unreachable),
        "unreachable_repositions": unreachable,
        "trajectory": traj,
        "cycle_time_s": cycle_time,
    }


if __name__ == '__main__':
    kin = DeltaKinematics()
    cam = CameraInterface()
    coupling = BaseCoupling(kin)

    print("--- Joint-limit + collision-aware IK sanity check ---")
    for pt in [(0, 0, -150), (20, 30, -150), (0, 0, -60)]:
        print(pt, "->", kin.inverse(*pt))

    print("\n--- Full pipeline test ---")
    from test_data_generator import generate_camera_frame_detections
    detections = generate_camera_frame_detections(6, seed=3)
    result = run_pipeline(kin, cam, coupling, detections)
    print(f"Detected: {result['n_detected']}, Reachable: {result['n_reachable']}, "
          f"Unreachable: {result['n_unreachable']}")
    print(f"Estimated cycle time: {result['cycle_time_s']:.2f}s")

    with open(cfg.results_path("integration_pipeline_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerow(["n_detected", result["n_detected"]])
        w.writerow(["n_reachable", result["n_reachable"]])
        w.writerow(["n_unreachable", result["n_unreachable"]])
        w.writerow(["cycle_time_s", round(result["cycle_time_s"], 2)])

    print(f"\nSaved integration summary to: {cfg.results_path('integration_pipeline_summary.csv')}")
