"""
Ackermann Steering Navigation Controller for Webots (16m x 16m Arena - 9 Crop Rows, Maximum Coverage)

Autonomous Weed Removal Robot (AWRR) - Ackermann Kinematics Subsystem

Features:
1. Maximum Field Coverage Efficiency (9 Crop Rows):
   - Arena Walls: X = ±8.0m, Y = ±8.0m
   - 9 Crop Row Line Centerlines: Y = [-6.4, -4.8, -3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4m] (1.6m spacing)
   - Visual green crop row lines rendered in 3D Webots scene tree along row centerlines
   - Robot center tracks exact crop row centerlines via GPS + IMU guidance

2. Mathematically Exact Semicircular Dubins U-Turn Arcs:
   - Parametrization: theta in [0, pi]
   - East Headland (+X): tx = 5.0 + r*sin(theta), ty = y_mid - r*cos(theta) (Arc Peak X = 5.8m)
   - West Headland (-X): tx = -5.0 - r*sin(theta), ty = y_mid - r*cos(theta) (Arc Peak X = -5.8m)
   - Bumper Clearance: 1.3m physical clearance to outer walls at ±8.0m!

3. Vector Projection & Automatic Waypoint Advancement:
   - Checks dot product of robot forward vector and waypoint vector (dot_prod = dx*cos(yaw) + dy*sin(yaw))
   - Automatically advances to next waypoint if waypoint falls behind front axle, eliminating 360° donut loops.
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

# Hardware Steering Limit: ±44.5° (0.7767 rad) to prevent Webots motor limit warnings
MAX_STEER_ANGLE = math.radians(44.5)

MAX_WHEEL_VEL = 20.0    # Actuator maximum angular speed (rad/s)

# --- SPEED & NAVIGATION TUNING ---
CRUISE_SPEED = 0.65     # Linear speed in crop row (m/s)
TURN_SPEED = 0.40       # Safe turning speed in headland (m/s)
REACH_THRESHOLD = 0.40  # Waypoint acceptance radius (m)

# Physical Safety Envelope (meters from LiDAR origin)
HEADLAND_WALL_STOP_M = 0.30   # Emergency braking buffer in headland
INROW_WALL_STOP_M = 0.50      # Emergency braking buffer in crop row

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
    """
    # Clamp virtual center steering angle strictly to [-44.5°, +44.5°]
    delta_center = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_cmd_center))

    if abs(delta_center) < 1e-4:
        # Straight line trajectory
        steer_fl.setPosition(0.0)
        steer_fr.setPosition(0.0)
        wheel_w = v_cmd / WHEEL_RADIUS
        wheel_w = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, wheel_w))
        for m in motors:
            m.setVelocity(wheel_w)
        return 0.0, 0.0, [wheel_w]*4

    # Ackermann turning radius from rear axle center
    R = WHEELBASE / math.tan(abs(delta_center))
    
    # Inner and outer wheel steering angles
    delta_in = math.atan(WHEELBASE / (R - TRACK_WIDTH / 2.0))
    delta_out = math.atan(WHEELBASE / (R + TRACK_WIDTH / 2.0))

    # Clamp front wheel angles to hardware limit
    delta_in = min(MAX_STEER_ANGLE, delta_in)
    delta_out = min(MAX_STEER_ANGLE, delta_out)

    if delta_center > 0:
        # Left Turn: Left = Inner, Right = Outer
        steer_left = delta_in
        steer_right = delta_out
        v_rl = v_cmd * (1.0 - TRACK_WIDTH / (2.0 * R))
        v_rr = v_cmd * (1.0 + TRACK_WIDTH / (2.0 * R))
    else:
        # Right Turn: Right = Inner, Left = Outer
        steer_left = -delta_out
        steer_right = -delta_in
        v_rl = v_cmd * (1.0 + TRACK_WIDTH / (2.0 * R))
        v_rr = v_cmd * (1.0 - TRACK_WIDTH / (2.0 * R))

    steer_fl.setPosition(steer_left)
    steer_fr.setPosition(steer_right)

    # Angular speeds for front and rear wheels
    w_fl = (v_cmd / math.cos(abs(steer_left))) / WHEEL_RADIUS
    w_fr = (v_cmd / math.cos(abs(steer_right))) / WHEEL_RADIUS
    w_rl = v_rl / WHEEL_RADIUS
    w_rr = v_rr / WHEEL_RADIUS

    wheel_speeds = [w_fl, w_fr, w_rl, w_rr]
    wheel_speeds = [max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, ws)) for ws in wheel_speeds]

    for i in range(4):
        motors[i].setVelocity(wheel_speeds[i])

    return steer_left, steer_right, wheel_speeds


def generate_perfect_9row_field_path(field_w=16.0, field_h=16.0, row_spacing=1.6, headland_margin=3.0, num_turn_pts=10):
    """
    Generates exact 9 Crop Row Dubins Ackermann path for 16m x 16m arena:
    - 9 Crop Rows at Y = [-6.4, -4.8, -3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4m]
    - Row endpoints: X = ±5.0m
    - Mathematically exact East & West semicircular U-turn arcs (R = 0.8m, Peak X = ±5.8m)
    - Starts at (-5.0, -6.4), Ends at (+5.0, +6.4)
    """
    waypoints = []
    is_headland_turn = []

    y_rows = [-6.4, -4.8, -3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4]

    for i, y in enumerate(y_rows):
        if i % 2 == 0:
            # West to East (-5.0 to +5.0)
            xs = [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0]
            for x in xs:
                waypoints.append((x, y))
                is_headland_turn.append(False)

            # Semicircular U-Turn Arc in East Headland (+X)
            if i < len(y_rows) - 1:
                next_y = y_rows[i+1]
                y_mid = (y + next_y) / 2.0
                r = (next_y - y) / 2.0  # R = 0.8m
                
                for j in range(1, num_turn_pts + 1):
                    theta = (j / num_turn_pts) * math.pi
                    tx = 5.0 + r * math.sin(theta)
                    ty = y_mid - r * math.cos(theta)
                    waypoints.append((round(tx, 3), round(ty, 3)))
                    is_headland_turn.append(True)
        else:
            # East to West (+5.0 to -5.0)
            xs = [5.0, 3.0, 1.0, -1.0, -3.0, -5.0]
            for x in xs:
                waypoints.append((x, y))
                is_headland_turn.append(False)

            # Semicircular U-Turn Arc in West Headland (-X)
            if i < len(y_rows) - 1:
                next_y = y_rows[i+1]
                y_mid = (y + next_y) / 2.0
                r = (next_y - y) / 2.0  # R = 0.8m
                
                for j in range(1, num_turn_pts + 1):
                    theta = (j / num_turn_pts) * math.pi
                    tx = -5.0 - r * math.sin(theta)
                    ty = y_mid - r * math.cos(theta)
                    waypoints.append((round(tx, 3), round(ty, 3)))
                    is_headland_turn.append(True)

    return waypoints, is_headland_turn

waypoints, is_headland_turn = generate_perfect_9row_field_path()
current_wp_idx = 0

print(f"[Ackermann Nav 16m] Generated {len(waypoints)} continuous 9-row Dubins waypoints.")
print(f"[Ackermann Nav 16m] Start: {waypoints[0]} | End: {waypoints[-1]}")
print(f"[Ackermann Nav 16m] Steering Envelope: [-44.5°, +44.5°]. Rear Axle: Fixed.")

trajectory_log = []
step_count = 0
last_print_time = -PRINT_INTERVAL_S
dt = TIME_STEP / 1000.0

# --- MAIN CONTROLLER LOOP ---
while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()

    if current_wp_idx >= len(waypoints):
        print(f"[Ackermann Nav 16m] ALL {len(waypoints)} WAYPOINTS COMPLETE. Final Position Reached.")
        set_ackermann_drive(0.0, 0.0)

        os.makedirs("results", exist_ok=True)
        with open("results/ackermann_16m_dense_navigation_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "yaw", "steer_cmd_deg"])
            writer.writerows(trajectory_log)
        print(f"[Ackermann Nav 16m] Logged complete path trajectory to results/ackermann_16m_dense_navigation_log.csv")
        break

    # Read Sensors (Global Pose Estimation via GPS + IMU)
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

    # Vector projection: Check if target waypoint is behind vehicle front axle line
    dot_prod = dx * math.cos(yaw) + dy * math.sin(yaw)
    in_turn = is_headland_turn[current_wp_idx]

    # Advancement Criteria: Close enough OR (behind vehicle AND dist < 0.8m)
    if dist < REACH_THRESHOLD or (dot_prod < 0 and dist < 0.80) or (in_turn and dist < 0.50):
        current_wp_idx += 1
        continue

    # Pure Pursuit Steering Control
    target_heading = math.atan2(dy, dx)
    heading_error = target_heading - yaw
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    # Steering Command Calculation & Strict Clamping [-44.5°, +44.5°]
    K_p_steer = 1.3 if in_turn else 1.1
    steer_cmd = K_p_steer * heading_error
    steer_cmd = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_cmd))

    # LiDAR Wall Distance Perception
    range_image = lidar.getRangeImage()
    min_front_range = float('inf')
    if range_image:
        n = len(range_image)
        fov = lidar.getFov()
        for i, r in enumerate(range_image):
            if math.isnan(r) or math.isinf(r) or r < lidar.getMinRange():
                continue
            angle = (fov / 2.0) - (i / (n - 1)) * fov
            if abs(angle) < math.radians(20):
                if r < min_front_range:
                    min_front_range = r

    # Determine Base Speed
    base_speed = TURN_SPEED if in_turn else CRUISE_SPEED

    # Context-Aware Wall Collision Protection
    stop_buffer = HEADLAND_WALL_STOP_M if in_turn else INROW_WALL_STOP_M
    slow_dist = stop_buffer + 0.40

    speed_cmd = base_speed
    if min_front_range < slow_dist:
        speed_scale = max(0.0, (min_front_range - stop_buffer) / (slow_dist - stop_buffer))
        speed_cmd *= speed_scale
        if min_front_range < stop_buffer:
            speed_cmd = 0.0
            print(f"[WALL ALARM] Boundary wall at {min_front_range:.2f}m. Emergency Halt.")

    # Apply Ackermann Steering Dynamics
    s_left, s_right, w_speeds = set_ackermann_drive(speed_cmd, steer_cmd)

    if (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        mode_str = "HEADLAND" if in_turn else "CROP_ROW"
        print(f"t={sim_time:.1f}s | wp={current_wp_idx}/{len(waypoints)} [{mode_str}] | pos=({pos[0]:.2f}, {pos[1]:.2f}) | dist={dist:.2f}m | steer={math.degrees(steer_cmd):.1f}° | wall={min_front_range:.2f}m")
