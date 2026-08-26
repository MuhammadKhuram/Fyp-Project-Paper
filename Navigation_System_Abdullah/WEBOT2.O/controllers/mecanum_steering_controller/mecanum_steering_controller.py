"""
Mecanum Pure Crab-Walk Navigation Controller for Webots (16m x 16m Arena - 9 Crop Rows)

Autonomous Weed Removal Robot (AWRR) - Mecanum Kinematics Subsystem

Authoritative Kinematics Formula (matching mecanum_milestoneD_crab.py):
- In-Row Driving (Forward/Reverse along X):
  set_mecanum_velocity(vx_body, 0.0, omega) -> fl=+, fr=+, rl=+, rr=+
- Headland Crab Strafing (Lateral along Y):
  vy_world = CRAB_SPEED if dy > 0 else -CRAB_SPEED
  vx_body = -vy_world * math.sin(yaw)
  vy_body = -vy_world * math.cos(yaw)
  set_mecanum_velocity(vx_body, vy_body, omega) -> fl=+, fr=-, rl=-, rr=+
"""

from controller import Robot
import math
import csv
import os

# --- VEHICLE GEOMETRY & KINEMATICS ---
TIME_STEP = 32

WHEEL_RADIUS = 0.075    # r: Wheel radius (m)
WHEELBASE = 0.60        # L: Wheelbase front-to-rear axle (m)
TRACK_WIDTH = 0.64      # W: Track width left-to-right (m)
L_W_SUM = (WHEELBASE + TRACK_WIDTH) / 2.0  # 0.62m

MAX_WHEEL_VEL = 20.0    # Actuator maximum angular speed (rad/s)

# --- SPEED & NAVIGATION TUNING ---
CRUISE_SPEED = 0.65     # Linear speed in crop row (m/s)
CRAB_SPEED = 0.45       # Lateral crab-strafing speed in headland (m/s)
REACH_THRESHOLD = 0.30  # Waypoint acceptance radius (m)

# Logging
LOG_EVERY_N_STEPS = 16
PRINT_INTERVAL_S = 1.0

# --- ROBOT & SENSOR INITIALIZATION ---
robot = Robot()

# 1. Drive Motors
motor_names = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
motors = [robot.getDevice(name) for name in motor_names]
for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# 2. Perception & Sensors
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)
lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)


def set_mecanum_velocities(vx, vy, omega):
    """
    Exact Mecanum inverse kinematics matching mecanum_milestoneD_crab.py:
    fl = (vx - vy - L_W_SUM * omega) / WHEEL_RADIUS
    fr = (vx + vy + L_W_SUM * omega) / WHEEL_RADIUS
    rl = (vx + vy - L_W_SUM * omega) / WHEEL_RADIUS
    rr = (vx - vy + L_W_SUM * omega) / WHEEL_RADIUS
    """
    fl = (vx - vy - L_W_SUM * omega) / WHEEL_RADIUS
    fr = (vx + vy + L_W_SUM * omega) / WHEEL_RADIUS
    rl = (vx + vy - L_W_SUM * omega) / WHEEL_RADIUS
    rr = (vx - vy + L_W_SUM * omega) / WHEEL_RADIUS

    wheel_speeds = [fl, fr, rl, rr]
    max_speed = max(abs(s) for s in wheel_speeds)
    if max_speed > MAX_WHEEL_VEL:
        scale = MAX_WHEEL_VEL / max_speed
        wheel_speeds = [s * scale for s in wheel_speeds]

    for i in range(4):
        motors[i].setVelocity(wheel_speeds[i])

    return wheel_speeds


def generate_9row_crab_waypoints():
    """
    Generates exact 9 Crop Row Crab-Walk waypoints:
    - 9 Crop Rows at Y = [-6.4, -4.8, -3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4m]
    - Row endpoints: X = ±5.0m
    """
    waypoints = []
    y_rows = [-6.4, -4.8, -3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4]

    for i, y in enumerate(y_rows):
        if i % 2 == 0:
            # West-to-East Row: (-5.0, Y_i) -> (+5.0, Y_i)
            waypoints.append((-5.0, y))
            waypoints.append((5.0, y))
        else:
            # East-to-West Row: (+5.0, Y_i) -> (-5.0, Y_i)
            waypoints.append((5.0, y))
            waypoints.append((-5.0, y))

    return waypoints

def is_row_switch_target(idx):
    # Segment driving TOWARD waypoint idx is a row switch (pure lateral move)
    # when idx is even and not the very first target (idx == 0 is initial spawn).
    return idx % 2 == 0 and idx > 0

waypoints = generate_9row_crab_waypoints()
current_wp_idx = 0

print(f"[Mecanum Crab Nav] Generated {len(waypoints)} waypoints for 9-row Crab-Walk.")
print(f"[Mecanum Crab Nav] Start: {waypoints[0]} | End: {waypoints[-1]}")

trajectory_log = []
step_count = 0
last_print_time = -PRINT_INTERVAL_S
dt = TIME_STEP / 1000.0

# --- MAIN CONTROLLER LOOP ---
while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()

    if current_wp_idx >= len(waypoints):
        print(f"[Mecanum Crab Nav] ALL {len(waypoints)} WAYPOINTS COMPLETE. Final Position Reached.")
        set_mecanum_velocities(0.0, 0.0, 0.0)

        os.makedirs("results", exist_ok=True)
        with open("results/mecanum_16m_dense_navigation_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "yaw", "steer_cmd_deg"])
            writer.writerows(trajectory_log)
        print(f"[Mecanum Crab Nav] Logged trajectory to results/mecanum_16m_dense_navigation_log.csv")
        break

    # Read Sensors
    pos = gps.getValues()
    rpy = imu.getRollPitchYaw()
    yaw = rpy[2]

    if math.isnan(pos[0]) or math.isnan(pos[1]) or math.isnan(yaw):
        continue

    tx, ty = waypoints[current_wp_idx]
    dx = tx - pos[0]
    dy = ty - pos[1]
    dist = math.hypot(dx, dy)

    # Log Trajectory
    step_count += 1
    if step_count % LOG_EVERY_N_STEPS == 0:
        trajectory_log.append((sim_time, pos[0], pos[1], yaw, 0.0))

    # Waypoint Advancement Check
    if dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[Mecanum Crab Nav] Reached Waypoint {current_wp_idx}/{len(waypoints)}")
        continue

    # Navigation Control Logic (matching mecanum_milestoneD_crab.py)
    if is_row_switch_target(current_wp_idx):
        # --- CRAB MODE: Pure Lateral Strafe along Y, hold heading at 0.0° ---
        vy_world = CRAB_SPEED if dy > 0 else -CRAB_SPEED
        vx_body = -vy_world * math.sin(yaw)
        vy_body = -vy_world * math.cos(yaw)

        heading_error = 0.0 - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        omega = max(-1.0, min(1.0, 2.0 * heading_error))

        set_mecanum_velocities(vx_body, vy_body, omega)
        mode_str = "CRAB_HEADLAND"
    else:
        # --- IN-ROW MODE: Drive along X ---
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        omega = max(-1.0, min(1.0, 1.5 * heading_error))
        speed_cap = CRUISE_SPEED if abs(heading_error) < math.radians(20) else 0.40
        forward_speed = min(speed_cap, dist) * max(0.0, math.cos(heading_error))
        if dx < 0:
            forward_speed = -min(speed_cap, dist)

        set_mecanum_velocities(forward_speed, 0.0, omega)
        mode_str = "CROP_ROW"

    if (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        print(f"t={sim_time:.1f}s | wp={current_wp_idx}/{len(waypoints)} [{mode_str}] | pos=({pos[0]:.2f}, {pos[1]:.2f}) | dist={dist:.2f}m | yaw={math.degrees(yaw):.1f}°")
