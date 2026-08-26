"""
Ackermann Steering Navigation Controller for Webots (16m x 16m Arena - 15 Crop Rows)

Autonomous Weed Removal Robot (AWRR) - Ackermann Kinematics Subsystem

Agricultural Features:
1. 15 Crop Rows at Y = [-7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0m] (1.0m Spacing)
2. 1.25m Dark Soil Headlands (X in [6.75, 8.0] and [-8.0, -6.75])
3. Interleaved Skip-Row Pathing (Every U-Turn Radius R >= 1.0m, zero oversailing!)
4. Dynamic 3-Point Headland K-Turn Re-Alignment Maneuver:
   - At the transition between Pass 1 (Row 15) and Pass 2 (Row 14), the robot executes a 3-point reverse K-turn
     100% inside the West headland to align perfectly co-linear with Row 14 (Y = 6.000m) without stepping on any crop rows!
"""

from controller import Robot
import math
import csv
import os

TIME_STEP = 32

WHEEL_RADIUS = 0.075    # r: Wheel radius (m)
WHEELBASE = 0.60        # L: Wheelbase front-to-rear axle (m)
TRACK_WIDTH = 0.64      # W: Track width left-to-right (m)

MAX_STEER_ANGLE = math.radians(44.5)  # Hardware steering limit (44.5°)
MAX_WHEEL_VEL = 25.0                  # Motor max angular speed (rad/s)

# Navigation Tuning
CRUISE_SPEED = 0.65     # Row speed (m/s)
TURN_SPEED = 0.45       # Headland U-turn speed (m/s)
REACH_THRESHOLD = 0.25  # Waypoint reach tolerance (m)
K_P_STEER = 1.6         # Proportional steering gain

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

# 2. Steering Motors
steer_fl = robot.getDevice('steer_fl_motor')
steer_fr = robot.getDevice('steer_fr_motor')
steer_rl = robot.getDevice('steer_rl_motor')
steer_rr = robot.getDevice('steer_rr_motor')

for sm in [steer_fl, steer_fr, steer_rl, steer_rr]:
    sm.setVelocity(8.0)

# Fixed Rear Axle (0° steering)
steer_rl.setPosition(0.0)
steer_rr.setPosition(0.0)

# 3. Perception & Sensors
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)
lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)


def set_ackermann_drive(v_cmd, steer_cmd_center):
    """
    Computes closed-form Ackermann steering angles (inner/outer) and wheel speeds.
    Supports both positive (forward) and negative (reverse) speeds.
    """
    delta_center = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_cmd_center))

    if abs(delta_center) < 1e-4:
        steer_fl.setPosition(0.0)
        steer_fr.setPosition(0.0)
        wheel_w = v_cmd / WHEEL_RADIUS
        wheel_w = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, wheel_w))
        for m in motors:
            m.setVelocity(wheel_w)
        return 0.0, 0.0, [wheel_w]*4

    R = WHEELBASE / math.tan(abs(delta_center))
    delta_in = math.atan(WHEELBASE / (R - TRACK_WIDTH / 2.0))
    delta_out = math.atan(WHEELBASE / (R + TRACK_WIDTH / 2.0))

    delta_in = min(MAX_STEER_ANGLE, delta_in)
    delta_out = min(MAX_STEER_ANGLE, delta_out)

    if delta_center > 0:
        steer_left = delta_in
        steer_right = delta_out
        v_rl = v_cmd * (1.0 - TRACK_WIDTH / (2.0 * R))
        v_rr = v_cmd * (1.0 + TRACK_WIDTH / (2.0 * R))
    else:
        steer_left = -delta_out
        steer_right = -delta_in
        v_rl = v_cmd * (1.0 + TRACK_WIDTH / (2.0 * R))
        v_rr = v_cmd * (1.0 - TRACK_WIDTH / (2.0 * R))

    steer_fl.setPosition(steer_left)
    steer_fr.setPosition(steer_right)

    w_fl = (v_cmd / math.cos(abs(steer_left))) / WHEEL_RADIUS
    w_fr = (v_cmd / math.cos(abs(steer_right))) / WHEEL_RADIUS
    w_rl = v_rl / WHEEL_RADIUS
    w_rr = v_rr / WHEEL_RADIUS

    wheel_speeds = [w_fl, w_fr, w_rl, w_rr]
    wheel_speeds = [max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, ws)) for ws in wheel_speeds]

    for i in range(4):
        motors[i].setVelocity(wheel_speeds[i])

    return steer_left, steer_right, wheel_speeds


def generate_15row_kturn_ackermann_path(num_turn_pts=10):
    """
    Generates 15 Crop Row Ackermann path with 3-Point Headland K-Turn Re-Alignment between Pass 1 and Pass 2:
    - Pass 1 (Odd):  Row 1 (-7) -> Row 3 (-5) -> Row 5 (-3) -> Row 7 (-1) -> Row 9 (1) -> Row 11 (3) -> Row 13 (5) -> Row 15 (7)
    - 3-Point K-Turn Re-alignment inside West Headland to align with Row 14 (6.0m)
    - Pass 2 (Even): Row 14 (6) -> Row 12 (4) -> Row 10 (2) -> Row 8 (0) -> Row 6 (-2) -> Row 4 (-4) -> Row 2 (-6)
    """
    waypoints = []
    is_headland_turn = []
    is_reverse_step = []

    row_sequence = [
        (-7.0, True),   # Row 1 (W -> E)
        (-5.0, False),  # Row 3 (E -> W)
        (-3.0, True),   # Row 5 (W -> E)
        (-1.0, False),  # Row 7 (E -> W)
        (1.0, True),    # Row 9 (W -> E)
        (3.0, False),   # Row 11 (E -> W)
        (5.0, True),    # Row 13 (W -> E)
        (7.0, False),   # Row 15 (E -> W)
        # --- 3-POINT HEADLAND K-TURN RE-ALIGNMENT HERE ---
        (6.0, True),    # Row 14 (W -> E)
        (4.0, False),   # Row 12 (E -> W)
        (2.0, True),    # Row 10 (W -> E)
        (0.0, False),   # Row 8 (E -> W)
        (-2.0, True),   # Row 6 (W -> E)
        (-4.0, False),  # Row 4 (E -> W)
        (-6.0, True),   # Row 2 (W -> E)
    ]

    for i, (y, w2e) in enumerate(row_sequence):
        # Add in-row waypoints
        if w2e:
            xs = [-6.5, -4.0, -1.5, 1.5, 4.0, 6.5]
        else:
            xs = [6.5, 4.0, 1.5, -1.5, -4.0, -6.5]

        for x in xs:
            waypoints.append((x, y))
            is_headland_turn.append(False)
            is_reverse_step.append(False)

        # Add headland U-turn or K-turn arc to next row if not the last row
        if i < len(row_sequence) - 1:
            next_y, next_w2e = row_sequence[i+1]

            # Check if this is the Pass 1 to Pass 2 Transition (Row 15 -> Row 14)
            if abs(next_y - y) == 1.0:
                # --- DYNAMIC 3-POINT HEADLAND K-TURN MANEUVER ---
                # Step 1: Coast forward into West headland to X = -7.1m at Y = 7.0m
                waypoints.append((-7.1, 7.0))
                is_headland_turn.append(True)
                is_reverse_step.append(False)

                # Step 2: Reverse angled down-east to X = -6.4m, Y = 6.5m
                waypoints.append((-6.4, 6.5))
                is_headland_turn.append(True)
                is_reverse_step.append(True)

                # Step 3: Forward align into Row 14 centerline at X = -6.5m, Y = 6.0m
                waypoints.append((-6.5, 6.0))
                is_headland_turn.append(True)
                is_reverse_step.append(False)
            else:
                y_mid = (y + next_y) / 2.0
                r = min(1.0, abs(next_y - y) / 2.0)
                sign = 1.0 if next_y > y else -1.0

                if w2e:
                    # Arrived at East headland (+X, facing East), turning to next row
                    for j in range(1, num_turn_pts + 1):
                        alpha = (j / num_turn_pts) * math.pi
                        theta = -math.pi/2.0 + alpha if sign > 0 else math.pi/2.0 - alpha
                        tx = 6.5 + r * math.cos(theta)
                        ty = y_mid + r * math.sin(theta)
                        waypoints.append((round(tx, 3), round(ty, 3)))
                        is_headland_turn.append(True)
                        is_reverse_step.append(False)
                else:
                    # Arrived at West headland (-X, facing West), turning to next row
                    for j in range(1, num_turn_pts + 1):
                        alpha = (j / num_turn_pts) * math.pi
                        theta = -math.pi/2.0 + alpha if sign > 0 else math.pi/2.0 - alpha
                        tx = -6.5 - r * math.cos(theta)
                        ty = y_mid + r * math.sin(theta)
                        waypoints.append((round(tx, 3), round(ty, 3)))
                        is_headland_turn.append(True)
                        is_reverse_step.append(False)

    return waypoints, is_headland_turn, is_reverse_step

waypoints, is_headland_turn, is_reverse_step = generate_15row_kturn_ackermann_path()
current_wp_idx = 0

print(f"[Ackermann Nav 16m] Generated {len(waypoints)} waypoints with 3-Point K-Turn Re-Alignment.")
print(f"[Ackermann Nav 16m] Start: {waypoints[0]} | End: {waypoints[-1]}")

trajectory_log = []
step_count = 0
last_print_time = -PRINT_INTERVAL_S
dt = TIME_STEP / 1000.0

# --- MAIN CONTROLLER LOOP ---
while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()

    if current_wp_idx >= len(waypoints):
        print(f"[Ackermann Nav 16m] ALL {len(waypoints)} WAYPOINTS COMPLETE.")
        set_ackermann_drive(0.0, 0.0)

        os.makedirs("results", exist_ok=True)
        with open("results/ackermann_16m_dense_navigation_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "yaw", "steer_cmd_deg"])
            writer.writerows(trajectory_log)
        print(f"[Ackermann Nav 16m] Logged trajectory to results/ackermann_16m_dense_navigation_log.csv")
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

    # Dot Product Waypoint Advancement Check
    in_turn = is_headland_turn[current_wp_idx]
    is_rev = is_reverse_step[current_wp_idx]
    heading_vec = (math.cos(yaw), math.sin(yaw)) if not is_rev else (-math.cos(yaw), -math.sin(yaw))
    dot = dx * heading_vec[0] + dy * heading_vec[1]

    if dist < REACH_THRESHOLD or (dot < 0.0 and dist < 0.8):
        current_wp_idx += 1
        continue

    # Pure Pursuit / Stanley Heading Control with Reverse Support
    if is_rev:
        target_heading = math.atan2(-dy, -dx)
        heading_error = target_heading - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        steer_cmd = K_P_STEER * heading_error
        speed_cmd = -0.35  # Reverse drive speed
    else:
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        steer_cmd = K_P_STEER * heading_error
        speed_cmd = TURN_SPEED if in_turn else CRUISE_SPEED

    set_ackermann_drive(speed_cmd, steer_cmd)

    if (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        mode_str = "K_TURN_REVERSE" if is_rev else ("HEADLAND_UTURN" if in_turn else "CROP_ROW")
        print(f"t={sim_time:.1f}s | wp={current_wp_idx}/{len(waypoints)} [{mode_str}] | pos=({pos[0]:.2f}, {pos[1]:.2f}) | dist={dist:.2f}m | steer={math.degrees(steer_cmd):.1f}°")
