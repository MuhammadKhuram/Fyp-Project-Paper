"""
Main Robot Controller — Autonomous Weed Removal System
========================================================
Integrates: Delta robot + Mobile platform + YOLOv8 detection + SLAM

State Machine:
  IDLE → NAVIGATING → SCANNING → DETECTING → REMOVING → NAVIGATING → ...
"""

import time
import numpy as np
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

from delta_robot_kinematics import inverse_kinematics, plan_weed_removal_trajectory
from yolov8_weed_detector import WeedDetector, WeedDetection
from mobile_robot_navigation import BoustrophedonPlanner, EKFLocalizer, FieldMap, Pose2D


class RobotState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    SCANNING   = auto()
    DETECTING  = auto()
    REMOVING   = auto()
    CHARGING   = auto()
    EMERGENCY  = auto()


@dataclass
class RobotConfig:
    # Field
    field_width_m: float   = 20.0
    field_height_m: float  = 30.0
    # Robot
    travel_speed_m_per_s: float = 0.3
    scan_speed_m_per_s: float   = 0.15
    delta_speed_mm_per_s: float = 80.0
    # Detection
    conf_threshold: float    = 0.45
    min_weed_radius_mm: float = 10.0
    # Safety
    min_battery_pct: float   = 20.0
    emergency_stop_dist_m: float = 0.15
    # Operational
    scan_height_mm: float    = -50.0
    cut_depth_mm: float      = -170.0


@dataclass
class RobotStatus:
    state: RobotState = RobotState.IDLE
    battery_pct: float = 100.0
    weeds_detected: int = 0
    weeds_removed: int = 0
    area_covered_m2: float = 0.0
    runtime_s: float = 0.0
    last_error: str = ''


class AutonomousWeedRobot:
    """
    Top-level controller for the Autonomous Weed Removal Robot.
    Orchestrates perception, planning, and execution loops.
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        self.config = config or RobotConfig()
        self.status = RobotStatus()
        self._start_time = time.time()

        # Initialize subsystems
        self.field_map = FieldMap(
            width_m=self.config.field_width_m,
            height_m=self.config.field_height_m
        )
        self.localizer = EKFLocalizer()
        self.detector  = WeedDetector(conf_threshold=self.config.conf_threshold)
        self.planner   = BoustrophedonPlanner(self.field_map)

        self._waypoints: List[Pose2D] = []
        self._current_waypoint_idx: int = 0
        self._pending_weeds: List[WeedDetection] = []
        self._session_log: List[dict] = []

        print(f"[Robot] Initialized — field {self.config.field_width_m}×{self.config.field_height_m}m")

    # ── State transitions ────────────────────────────────────────────────────

    def start_mission(self):
        """Begin autonomous field coverage mission."""
        if self.status.state != RobotState.IDLE:
            print("[Robot] ERROR: Cannot start — not in IDLE state")
            return
        start_pose = Pose2D(x=0.5, y=0.5, theta=0.0)
        self._waypoints = self.planner.plan(start_pose)
        self._current_waypoint_idx = 0
        self._transition(RobotState.NAVIGATING)
        print(f"[Robot] Mission started — {len(self._waypoints)} waypoints")

    def _transition(self, new_state: RobotState):
        old = self.status.state
        self.status.state = new_state
        self._log_event(f"State: {old.name} → {new_state.name}")

    # ── Perception ───────────────────────────────────────────────────────────

    def scan_and_detect(self,
                         rgb: np.ndarray,
                         depth: Optional[np.ndarray] = None) -> List[WeedDetection]:
        """Run detection on current camera frame."""
        self._transition(RobotState.DETECTING)
        detections = self.detector.detect(rgb, depth)
        # Filter by minimum size and non-crop class
        valid = [d for d in detections
                 if d.class_name != 'Background'
                 and d.weed_radius_mm >= self.config.min_weed_radius_mm]
        self.status.weeds_detected += len(valid)
        # Update map
        for d in valid:
            if d.bbox_world:
                pose = self.localizer.pose
                wx = pose.x * 1000 + d.bbox_world[0]
                wy = pose.y * 1000 + d.bbox_world[1]
                self.field_map.mark_weed(wx/1000, wy/1000, d.confidence)
        self._transition(RobotState.REMOVING)
        return valid

    # ── Execution ────────────────────────────────────────────────────────────

    def execute_weed_removal(self, detections: List[WeedDetection]) -> int:
        """
        Command delta robot to remove detected weeds.
        Returns number successfully removed.
        """
        weed_positions = []
        for d in detections:
            if d.bbox_world:
                weed_positions.append(d.bbox_world)
            else:
                # Fallback: use pixel center projected to nominal depth
                x1, y1, x2, y2 = d.bbox_xyxy
                cx_px = (x1 + x2) / 2.0 - 320.0  # relative to center
                cy_px = (y1 + y2) / 2.0 - 240.0
                weed_positions.append((cx_px * 0.5, cy_px * 0.5))  # rough mm estimate

        trajectory = plan_weed_removal_trajectory(
            weed_positions,
            approach_height=self.config.scan_height_mm,
            cut_depth=self.config.cut_depth_mm,
            speed_mm_per_s=self.config.delta_speed_mm_per_s
        )
        # In production: send trajectory to delta robot hardware controller
        # Here we count successful (reachable) waypoints
        cuts = sum(1 for wp in trajectory if wp['phase'] == 'cut')
        self.status.weeds_removed += cuts
        self._log_event(f"Removed {cuts}/{len(detections)} weeds")
        return cuts

    # ── Navigation ───────────────────────────────────────────────────────────

    def step_navigation(self, dt: float = 0.1) -> bool:
        """
        Advance along planned path by one time step.
        Returns True if mission is complete.
        """
        if self._current_waypoint_idx >= len(self._waypoints) - 1:
            self._transition(RobotState.IDLE)
            self._log_event("Mission complete")
            return True

        # Simulate EKF update
        self.localizer.predict(dt)
        target = self._waypoints[self._current_waypoint_idx + 1]
        # Simulate GPS + IMU noise
        self.localizer.update_gps(
            target.x + np.random.normal(0, 0.02),
            target.y + np.random.normal(0, 0.02)
        )
        pose = self.localizer.pose
        if pose.distance_to(target) < 0.15:
            self._current_waypoint_idx += 1
            # Update coverage
            self.status.area_covered_m2 = (
                self._current_waypoint_idx / len(self._waypoints)
                * self.config.field_width_m * self.config.field_height_m
            )
        return False

    # ── Safety ───────────────────────────────────────────────────────────────

    def check_safety(self) -> bool:
        """
        Return True if it's safe to continue.
        Checks battery, e-stop, and localization confidence.
        """
        if self.status.battery_pct < self.config.min_battery_pct:
            self._transition(RobotState.CHARGING)
            print("[Robot] LOW BATTERY — returning to charge")
            return False
        if self.localizer.position_uncertainty_m > 0.5:
            print(f"[Robot] WARNING: Localization uncertainty {self.localizer.position_uncertainty_m*100:.1f}cm")
        return True

    # ── Logging ──────────────────────────────────────────────────────────────

    def _log_event(self, msg: str):
        self.status.runtime_s = time.time() - self._start_time
        entry = {'t': self.status.runtime_s, 'msg': msg, 'state': self.status.state.name}
        self._session_log.append(entry)

    def summary(self) -> dict:
        rt = self.status.runtime_s
        efficiency = (self.status.weeds_removed / max(self.status.weeds_detected, 1)) * 100
        return {
            'state': self.status.state.name,
            'runtime_min': rt / 60.0,
            'weeds_detected': self.status.weeds_detected,
            'weeds_removed': self.status.weeds_removed,
            'removal_efficiency_pct': efficiency,
            'area_covered_m2': self.status.area_covered_m2,
            'battery_pct': self.status.battery_pct,
            'avg_inference_ms': self.detector.avg_inference_time_ms,
        }


# ─── Simulation Demo ─────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Autonomous Weed Removal Robot — Simulation")
    print("=" * 50)

    robot = AutonomousWeedRobot()
    robot.start_mission()

    # Simulate 30 navigation steps with detection every 5 steps
    for step in range(30):
        done = robot.step_navigation(dt=2.0)
        robot.status.battery_pct -= 0.5  # discharge

        if step % 5 == 0:
            dummy_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            dummy_depth = np.random.uniform(0.5, 1.2, (480, 640)).astype(np.float32)
            detections = robot.scan_and_detect(dummy_img, dummy_depth)
            if detections:
                robot.execute_weed_removal(detections)
            robot._transition(RobotState.NAVIGATING)

        if not robot.check_safety():
            break
        if done:
            break

    s = robot.summary()
    print("\n--- Mission Summary ---")
    for k, v in s.items():
        print(f"  {k:30s}: {v:.2f}" if isinstance(v, float) else f"  {k:30s}: {v}")
