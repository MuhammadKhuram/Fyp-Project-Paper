"""
4-Wheel Crab-Walk Navigation Controller for Webots (16m x 16m Arena - 15 Crop Rows)

Autonomous Weed Removal Robot (AWRR) - Crab Kinematics Subsystem

Physical Crab-Walk Mechanics:
1. Row 1, 3, 5, 7, 9, 11, 13, 15 (West-to-East along +X):
   - Steering Motors (steer_fl, steer_fr, steer_rl, steer_rr): Set to 0.0 rad (Pointing East).
   - Drive Motors: Set to +8.67 rad/s (+0.65 m/s forward).
   - Robot drives East from (-6.8m, Y_i) to (+6.8m, Y_i).

2. East Headland Crab Shift (+Y from Y_i to Y_{i+1}):
   - Steering Motors (steer_fl, steer_fr, steer_rl, steer_rr): Set to +1.5708 rad (+90°, Pointing North).
   - Drive Motors: Set to +6.00 rad/s (+0.45 m/s forward).
   - Robot rolls smoothly North along +Y inside the dark headland strip from (+6.8m, Y_i) to (+6.8m, Y_{i+1}).

3. Row 2, 4, 6, 8, 10, 12, 14 (East-to-West along -X):
   - Steering Motors (steer_fl, steer_fr, steer_rl, steer_rr): Set to +3.14159 rad (+180°, Pointing West).
   - Drive Motors: Set to +8.67 rad/s (+0.65 m/s forward).
   - Robot drives West from (+6.8m, Y_i) to (-6.8m, Y_i) without turning the robot body at all!

4. West Headland Crab Shift (+Y from Y_i to Y_{i+1}):
   - Steering Motors (steer_fl, steer_fr, steer_rl, steer_rr): Set to +1.5708 rad (+90°, Pointing North).
   - Drive Motors: Set to +6.00 rad/s (+0.45 m/s forward).
   - Robot rolls smoothly North along +Y inside the dark headland strip from (-6.8m, Y_i) to (-6.8m, Y_{i+1}).
"""

from controller import Robot
import math
import csv
import os

TIME_STEP = 32

WHEEL_RADIUS = 0.075    # r: Wheel radius (m)
WHEELBASE = 0.60        # L: Wheelbase (m)
TRACK_WIDTH = 0.64      # W: Track width (m)

CRUISE_SPEED = 0.65     # Max row speed (m/s)
CRAB_SPEED = 0.45       # Max headland crab speed (m/s)
REACH_THRESHOLD = 0.25  # Waypoint reach tolerance (m)

# Logging
LOG_EVERY_N_STEPS = 16
PRINT_INTERVAL_S = 1.0

robot = Robot()

# 1. Steering Motors (steer_fl, steer_fr, steer_rl, steer_rr)
steer_fl = robot.getDevice('steer_fl_motor')
steer_fr = robot.getDevice('steer_fr_motor')
steer_rl = robot.getDevice('steer_rl_motor')
steer_rr = robot.getDevice('steer_rr_motor')
steer_motors = [steer_fl, steer_fr, steer_rl, steer_rr]

for sm in steer_motors:
    sm.setVelocity(12.0)

# 2. Drive Motors (wheel_fl, wheel_fr, wheel_rl, wheel_rr)
motor_names = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
drive_motors = [robot.getDevice(name) for name in motor_names]
for dm in drive_motors:
    dm.setPosition(float('inf'))
    dm.setVelocity(0.0)

# 3. Perception & Sensors
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)
lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)


def set_crab_drive(steer_angle_rad, drive_speed_m_s):
    """
    Sets all 4 wheel steering angles and drive motor velocities.
    """
    for sm in steer_motors:
        sm.setPosition(steer_angle_rad)

    w_speed = drive_speed_m_s / WHEEL_RADIUS
    w_speed = max(-25.0, min(25.0, w_speed))
    for dm in drive_motors:
        dm.setVelocity(w_speed)


def generate_15row_headland_crab_waypoints():
    """
    Generates 15 Crop Row Crab-Walk waypoints extending into 1.25m headlands:
    - 15 Crop Rows at Y = [-7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0m]
    - Row endpoints extended into dark headland strips at X = ±6.8m
    """
    waypoints = []
    y_rows = [-7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

    for i, y in enumerate(y_rows):
        if i % 2 == 0:
            # West-to-East: Drive from (-6.8, Y_i) to (+6.8, Y_i) [Headland to Headland]
            waypoints.append((-6.8, y))
            waypoints.append((6.8, y))
        else:
            # East-to-West: Drive from (+6.8, Y_i) to (-6.8, Y_i) [Headland to Headland]
            waypoints.append((6.8, y))
            waypoints.append((-6.8, y))

    return waypoints

def is_headland_crab_segment(idx):
    # Segment driving TOWARD waypoint idx is a headland crab shift along +Y
    # when idx is even and not the initial spawn.
    return idx % 2 == 0 and idx > 0

waypoints = generate_15row_headland_crab_waypoints()
current_wp_idx = 0

print(f"[Crab Nav 15m] Generated {len(waypoints)} waypoints for 15-row Crab-Walk.")
print(f"[Crab Nav 15m] Start: {waypoints[0]} | End: {waypoints[-1]}")

trajectory_log = []
step_count = 0
last_print_time = -PRINT_INTERVAL_S
dt = TIME_STEP / 1000.0

# --- MAIN CONTROLLER LOOP ---
while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()

    if current_wp_idx >= len(waypoints):
        print(f"[Crab Nav 15m] ALL {len(waypoints)} WAYPOINTS COMPLETE. Final Position Reached.")
        set_crab_drive(0.0, 0.0)

        os.makedirs("results", exist_ok=True)
        with open("results/mecanum_16m_dense_navigation_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "yaw", "steer_cmd_deg"])
            writer.writerows(trajectory_log)
        print(f"[Crab Nav 15m] Logged trajectory to results/mecanum_16m_dense_navigation_log.csv")
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

    # Waypoint Advancement Check (0.25m threshold)
    if dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[Crab Nav 15m] Reached Waypoint {current_wp_idx}/{len(waypoints)} at pos=({pos[0]:.2f}, {pos[1]:.2f})")
        continue

    # Navigation Control Logic
    if is_headland_crab_segment(current_wp_idx):
        # --- HEADLAND CRAB SHIFT: Rotate wheels 90° (+1.5708 rad), roll North along +Y inside headland ---
        steer_target = 1.5708  # 90° (Pointing North)
        drive_target = CRAB_SPEED if dy > 0 else -CRAB_SPEED

        set_crab_drive(steer_target, drive_target)
        mode_str = "CRAB_HEADLAND"
    else:
        # --- IN-ROW CROP PASS: 0° (Pointing East) or 180° (Pointing West) ---
        if dx > 0:
            # West-to-East: Steer = 0.0 rad (East), Drive = +0.65 m/s
            steer_target = 0.0
            drive_target = CRUISE_SPEED
        else:
            # East-to-West: Steer = 3.14159 rad (West), Drive = +0.65 m/s
            steer_target = math.pi
            drive_target = CRUISE_SPEED

        set_crab_drive(steer_target, drive_target)
        mode_str = "CROP_ROW"

    if (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        print(f"t={sim_time:.1f}s | wp={current_wp_idx}/{len(waypoints)} [{mode_str}] | pos=({pos[0]:.2f}, {pos[1]:.2f}) | dist={dist:.2f}m | yaw={math.degrees(yaw):.1f}°")
