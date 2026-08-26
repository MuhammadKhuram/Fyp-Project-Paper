"""
Milestone E - Advanced LiDAR-based Obstacle Avoidance Controller
with Crop-Preservation Boundaries (Option A) and Uncertainty-Adaptive Safety Envelope (Option B)

Implements:
- Waypoint-following (Boustrophedon pattern)
- Trajectory logging (Milestone B)
- Complementary Filter localization with noise injection + GPS Blackout (Milestone C)
- Uncertainty-Adaptive APF: Safety envelope inflates as localization confidence drifts.
- Crop-Preservation Constraints: Clamps lateral swerve vectors to prevent crop row intrusion,
  forcing a safe emergency halt if the corridor is completely blocked.
"""

from controller import Robot
import math
import csv
import os
import random

# --- CONFIG ---
TIME_STEP = 32

WHEEL_RADIUS = 0.075
L = 0.3
W = 0.32
MAX_WHEEL_VEL = 20.0

REACH_THRESHOLD = 0.15
MAX_SPEED = 0.3
CRUISE_SPEED = 0.6
STRAIGHT_HEADING_THRESHOLD = math.radians(5)
K_HEADING = 5.0
OMEGA_MAX = 1.5

PRINT_INTERVAL_S = 1.0
LOG_EVERY_N_STEPS = 16

# --- Milestone C: localization noise + filter config ---
GPS_NOISE_SIGMA = 0.03
BLACKOUT_START_S = 100.0
BLACKOUT_END_S = 140.0
GPS_CORRECTION_WEIGHT = 0.4

# --- Milestone E: Advanced Obstacle Avoidance config ---
BASE_AVOID_THRESHOLD_M = 1.3  # base distance to start avoiding obstacles (tuned)
CRITICAL_STOP_M = 0.45        # base stopping buffer
REPULSIVE_K = 1.5             # gain factor for repulsive force (tuned to prevent over-correction)
FRONT_FOV_HALF_DEG = 12.0     # sector considered "directly in front" (narrowed to ignore crop walls)
LANE_HALF_WIDTH = 0.60        # max allowed lateral deviation from row center (in meters, tuned for 2.2m lanes)

DEBUG = True

# --- INIT ---
robot = Robot()

# Drive motors
motor_names = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
motors = [robot.getDevice(name) for name in motor_names]
for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# Steering motors
steer_names = ['steer_fl_motor', 'steer_fr_motor', 'steer_rl_motor', 'steer_rr_motor']
steer_motors = [robot.getDevice(name) for name in steer_names]
for sm in steer_motors:
    sm.setPosition(0.0)
    sm.setVelocity(10.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)

# Drive sensors
sensor_names = ['wheel_fl_sensor', 'wheel_fr_sensor', 'wheel_rl_sensor', 'wheel_rr_sensor']
sensors = [robot.getDevice(name) for name in sensor_names]
for s in sensors:
    s.enable(TIME_STEP)

# Steering sensors
steer_sensor_names = ['steer_fl_sensor', 'steer_fr_sensor', 'steer_rl_sensor', 'steer_rr_sensor']
steer_sensors = [robot.getDevice(name) for name in steer_sensor_names]
for ss in steer_sensors:
    ss.enable(TIME_STEP)

# Enable Lidar
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)

# --- SWERVE KINEMATICS ---
def set_swerve_velocity(vx, vy, omega):
    # Wheel coordinates (FL, FR, RL, RR)
    xs = [0.3, 0.3, -0.3, -0.3]
    ys = [0.32, -0.32, 0.32, -0.32]

    # Get current steering angles
    current_steer = [ss.getValue() for ss in steer_sensors]
    current_steer = [0.0 if math.isnan(a) else a for a in current_steer]

    speeds = []
    
    for i in range(4):
        # Target velocity vector for each wheel in robot frame
        vx_w = vx - ys[i] * omega
        vy_w = vy + xs[i] * omega
        
        speed = math.hypot(vx_w, vy_w)
        if speed > 0.01:
            target_angle = math.atan2(vy_w, vx_w)
        else:
            target_angle = current_steer[i]
            
        # Shortest-path angle wrapping steering optimization
        diff = target_angle - current_steer[i]
        diff = math.atan2(math.sin(diff), math.cos(diff))
        
        if abs(diff) > math.pi / 2.0:
            if diff > 0:
                diff -= math.pi
            else:
                diff += math.pi
            speed = -speed
            
        new_angle = current_steer[i] + diff
        wheel_vel = speed / WHEEL_RADIUS
        
        if abs(wheel_vel) > MAX_WHEEL_VEL:
            wheel_vel = math.copysign(MAX_WHEEL_VEL, wheel_vel)
            
        steer_motors[i].setPosition(new_angle)
        motors[i].setVelocity(wheel_vel)
        speeds.append(wheel_vel)
        
    return speeds

# --- BOUSTROPHEDON PATH WITH CIRCULAR U-TURNS ---
def generate_boustrophedon(field_w=10.0, field_h=10.0, stripe_width=2.2, margin=0.5, num_turn_points=8):
    waypoints = []
    wp_in_row = []
    
    x_start = -field_w / 2 + margin
    x_end = field_w / 2 - margin
    y_min = -field_h / 2 + margin
    y_max = field_h / 2 - margin

    num_stripes = int((field_h - 2 * margin) / stripe_width) + 1

    # spawn point transit
    waypoints.append((x_start, y_min))
    wp_in_row.append(False)

    for i in range(num_stripes):
        y = min(y_min + i * stripe_width, y_max)
        
        if i > 0:
            prev_x, prev_y = waypoints[-1]
            y_mid = (prev_y + y) / 2.0
            r = (y - prev_y) / 2.0
            
            is_at_end = (abs(prev_x - x_end) < 0.1)
            direction_sign = 1.0 if is_at_end else -1.0
            
            # Generate circular arc waypoints
            for j in range(1, num_turn_points + 1):
                alpha = -math.pi/2 + (j / num_turn_points) * math.pi
                tx = prev_x + direction_sign * r * math.cos(alpha)
                ty = y_mid + r * math.sin(alpha)
                waypoints.append((tx, ty))
                wp_in_row.append(False)
                
        row_target_x = x_start if (i % 2 == 1) else x_end
        waypoints.append((row_target_x, y))
        wp_in_row.append(True)

    return waypoints, wp_in_row

waypoints, wp_in_row = generate_boustrophedon()
current_wp_idx = 0

print(f"[Nav] Generated {len(waypoints)} waypoints")
print(f"[Nav] Running Advanced Milestone E: CP-UA Obstacle Avoidance (Swerve Drive)")

prev_sensor_vals = [0.0, 0.0, 0.0, 0.0]
dt = TIME_STEP / 1000.0

# --- LOGGING state (Milestone B) ---
trajectory_log = []
step_count = 0

# --- PRINT THROTTLE state ---
last_print_time = -PRINT_INTERVAL_S

# --- Localization filter + uncertainty state (Milestone C & E) ---
est_x, est_y = None, None
last_forward_speed, last_omega = 0.0, 0.0
localization_log = []
pos_uncertainty = GPS_NOISE_SIGMA

# --- MAIN LOOP ---
while robot.step(TIME_STEP) != -1:
    if current_wp_idx >= len(waypoints):
        if current_wp_idx == len(waypoints):
            print("[Nav] ALL WAYPOINTS COMPLETE")
            current_wp_idx += 1

            os.makedirs("results", exist_ok=True)

            with open("results/trajectory_log.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "x", "y", "heading"])
                writer.writerows(trajectory_log)
            print(f"[Nav] Logged {len(trajectory_log)} rows to results/trajectory_log.csv")

            with open("results/localization_log.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "gt_x", "gt_y", "noisy_gps_x", "noisy_gps_y",
                                  "est_x", "est_y", "in_blackout"])
                writer.writerows(localization_log)
            print(f"[Nav] Logged {len(localization_log)} rows to results/localization_log.csv")

        set_swerve_velocity(0, 0, 0)
        continue

    pos = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    
    # Skip loop iteration if sensor values are not initialized yet (NaN at startup)
    if math.isnan(pos[0]) or math.isnan(pos[1]) or math.isnan(yaw):
        continue

    tx, ty = waypoints[current_wp_idx]
    dx = tx - pos[0]
    dy = ty - pos[1]
    dist = math.hypot(dx, dy)

    sim_time = robot.getTime()

    # --- trajectory logging (Milestone B) ---
    step_count += 1
    if step_count % LOG_EVERY_N_STEPS == 0:
        trajectory_log.append((sim_time, pos[0], pos[1], yaw))

    # --- localization noise injection + complementary filter (Milestone C) ---
    gt_x, gt_y = pos[0], pos[1]
    noisy_gps_x = gt_x + random.gauss(0, GPS_NOISE_SIGMA)
    noisy_gps_y = gt_y + random.gauss(0, GPS_NOISE_SIGMA)

    in_blackout = BLACKOUT_START_S <= sim_time <= BLACKOUT_END_S

    # Track localization uncertainty growth/recovery (Milestone E - Option B)
    if in_blackout:
        pos_uncertainty += 0.035 * dt  # Uncertainty grows during blackout
    else:
        pos_uncertainty = GPS_NOISE_SIGMA  # Resets when GPS is restored

    if est_x is None:
        est_x, est_y = gt_x, gt_y
    else:
        pred_x = est_x + last_forward_speed * math.cos(yaw) * dt
        pred_y = est_y + last_forward_speed * math.sin(yaw) * dt

        if in_blackout:
            est_x, est_y = pred_x, pred_y
        else:
            est_x = (1 - GPS_CORRECTION_WEIGHT) * pred_x + GPS_CORRECTION_WEIGHT * noisy_gps_x
            est_y = (1 - GPS_CORRECTION_WEIGHT) * pred_y + GPS_CORRECTION_WEIGHT * noisy_gps_y

    localization_log.append((sim_time, gt_x, gt_y, noisy_gps_x, noisy_gps_y,
                              est_x, est_y, int(in_blackout)))

    if dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[Nav] Reached waypoint {current_wp_idx}/{len(waypoints)}")
        continue

    # --- Milestone E: LiDAR processing with Uncertainty-Adaptive safety envelope ---
    in_crop_row = wp_in_row[current_wp_idx]
    range_image = lidar.getRangeImage()
    n = len(range_image)
    fov = lidar.getFov()

    # Option B: Dynamically inflate safety envelope based on localization uncertainty
    # As uncertainty grows, start avoiding obstacles from further away and stop earlier
    avoid_threshold_m = BASE_AVOID_THRESHOLD_M + 0.3 * pos_uncertainty
    avoid_threshold_m = max(1.0, min(1.6, avoid_threshold_m))  # Keep within sensible limits (max 1.6m)

    stop_buffer_m = CRITICAL_STOP_M + 0.15 * pos_uncertainty
    stop_buffer_m = max(0.45, min(0.70, stop_buffer_m))  # Capped at 0.7m max

    min_left = avoid_threshold_m
    min_right = avoid_threshold_m
    min_front_range = float('inf')

    # Analyze each LiDAR beam to find the closest obstacle on the left and right halves
    for i, r in enumerate(range_image):
        if math.isnan(r) or math.isinf(r) or r < lidar.getMinRange():
            continue

        # Calculate beam angle relative to robot body (+x is straight ahead, +y is left)
        angle = (fov / 2.0) - (i / (n - 1)) * fov

        if r < avoid_threshold_m:
            if angle > 0:  # Left half
                if r < min_left:
                    min_left = r
            else:          # Right half
                if r < min_right:
                    min_right = r

        # Track the minimum distance of obstacles in the front cone
        if abs(math.degrees(angle)) <= FRONT_FOV_HALF_DEG:
            if r < min_front_range:
                min_front_range = r

    # Persist the evasion direction state across loop iterations to prevent lateral oscillations
    if 'evasion_dir' not in globals():
        global evasion_dir
        evasion_dir = None

    # Compute lateral repulsive force
    rep_force_y = 0.0
    if min_front_range < avoid_threshold_m:
        # If we are not currently evading, choose a direction and stick to it
        if evasion_dir is None:
            # Field-aware swerving: always swerve towards the inside of the field (away from boundary walls).
            # If we are in the bottom half of the field (ty < 0), swerve left (positive Y).
            # If we are in the top half (ty >= 0), swerve right (negative Y).
            if ty < 0:
                evasion_dir = "left"
            else:
                evasion_dir = "right"

        # Apply a strong, constant lateral repulsive force based on chosen state.
        # Note: in this Mecanum kinematics configuration, negative vy commands slide the robot left,
        # and positive vy commands slide it right.
        if evasion_dir == "left":
            rep_force_y = -1.2
        else:
            rep_force_y = 1.2
    else:
        # Clear the evasion state once the front path is clear AND we have passed the obstacle.
        # The obstacle in the simulation is placed at X = 0.0.
        # If we are traveling east (dx > 0), we have passed it when pos[0] >= 0.4.
        # If we are traveling west (dx < 0), we have passed it when pos[0] <= -0.4.
        # If we are in transition (not in crop row), we can clear it immediately.
        if not in_crop_row:
            evasion_dir = None
        else:
            is_east = (dx >= 0)
            if (is_east and pos[0] >= 0.4) or (not is_east and pos[0] <= -0.4):
                evasion_dir = None

    # Compute target steering direction in body frame
    if in_crop_row:
        # Lock heading parallel to the crop row axis (strictly parallel to X-axis)
        target_heading = 0.0 if dx >= 0 else math.pi
    else:
        # Allow turning to face the waypoint in transition/headland areas
        target_heading = math.atan2(dy, dx)
        
    heading_error = target_heading - yaw
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    target_force_x = 1.0  # Drive forward along local +x
    target_force_y = 0.0
    if in_crop_row:
        # Since heading correction is locked parallel to the row, we use lateral sliding (Mecanum)
        # to gently center the robot back onto the row line
        y_row = ty
        y_deviation = pos[1] - y_row
        target_force_y = 0.8 * y_deviation

    # Blend forces. For agricultural lanes, the obstacle repulsive force should only
    # slide the robot laterally (Y) to swerve. We do not allow rep_force_x to push
    # the robot backward, preventing forward-backward limit cycle oscillation.
    total_force_x = target_force_x
    total_force_y = target_force_y + REPULSIVE_K * rep_force_y

    # --- Option A: Crop-Preservation Corridor Clamping ---
    # Crop rows run horizontally (along constant Y lines).
    # If the robot is currently driving along a Boustrophedon stripe (odd waypoint index),
    # clamp lateral forces to prevent driving into the crops.
    in_crop_row = wp_in_row[current_wp_idx]
    boundary_limit_reached = False

    if in_crop_row:
        y_row = ty  # Center Y coordinate of current crop row
        y_deviation = pos[1] - y_row  # Lateral offset from crop lane center

        # Convert body-frame forces to world-frame to apply lane boundaries
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        world_force_x = total_force_x * cos_y - total_force_y * sin_y
        world_force_y = total_force_x * sin_y + total_force_y * cos_y

        # Define safety cushion from the crop wall
        safety_cushion_m = 0.05
        max_allowed_y = LANE_HALF_WIDTH - safety_cushion_m

        # If too far left, block any forces wanting to steer further left
        if y_deviation >= max_allowed_y:
            boundary_limit_reached = True
            if world_force_y > 0:
                world_force_y = 0.0
        # If too far right, block any forces wanting to steer further right
        elif y_deviation <= -max_allowed_y:
            boundary_limit_reached = True
            if world_force_y < 0:
                world_force_y = 0.0

        # Project modified forces back into the robot's body frame
        total_force_x = world_force_x * cos_y + world_force_y * sin_y
        total_force_y = -world_force_x * sin_y + world_force_y * cos_y

    # Calculate speed scaling factor based on frontal obstacles
    speed_factor = 1.0
    if min_front_range < avoid_threshold_m:
        speed_factor = max(0.0, (min_front_range - stop_buffer_m) / (avoid_threshold_m - stop_buffer_m))

    # Calculate desired forward (vx) and lateral (vy) velocities in body frame
    force_mag = math.hypot(total_force_x, total_force_y)
    speed_cap = CRUISE_SPEED if min_front_range >= avoid_threshold_m else MAX_SPEED
    
    # First-order low-pass filter to prevent lateral wiggling/oscillations
    if 'vy_body_filtered' not in globals():
        global vy_body_filtered
        vy_body_filtered = 0.0

    if force_mag > 0.0:
        # Decouple speed_factor from vy_body: speed_factor only slows/halts forward motion (vx_body).
        # vy_body (lateral evasion) remains active so the robot can slide laterally to escape.
        vx_body = (total_force_x / force_mag) * min(speed_cap, dist) * (speed_factor ** 2.0)
        vy_raw = (total_force_y / force_mag) * min(speed_cap, dist)
        
        # Low-pass filter to smooth out fast sign-switching lateral oscillations
        vy_body_filtered = 0.85 * vy_body_filtered + 0.15 * vy_raw
        vy_body = vy_body_filtered
    else:
        vx_body = 0.0
        vy_body_filtered = 0.85 * vy_body_filtered
        vy_body = 0.0

    # Steer command: align heading with the target waypoint
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_error))

    # EMERGENCY HALT & CROP-ROW ORIENTATION HOLD (Option A + B):
    if (in_crop_row and boundary_limit_reached and min_front_range < avoid_threshold_m * 0.8) or speed_factor <= 0.0:
        vx_body = 0.0
        # Stop lateral sliding only if we are at the crop boundary or critically close
        if boundary_limit_reached or min_front_range < stop_buffer_m:
            vy_body = 0.0
        
        if in_crop_row:
            omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_error))  # Maintain alignment
            if min_front_range < stop_buffer_m:
                print(f"[EMERGENCY] Obstacle too close ({min_front_range:.2f}m) & crop lane boundary reached. Halting in line.")
            else:
                print(f"[HALT] Swerve limit reached to protect crops. Obstacle blocking path. Awaiting clearance.")
        else:
            if min_front_range < stop_buffer_m:
                if rep_force_y > 0:
                    omega = -OMEGA_MAX  # Turn right
                else:
                    omega = OMEGA_MAX   # Turn left
                print(f"[EMERGENCY] Blocked. Rotating in place safely.")
            else:
                omega = 0.0
                print(f"[HALT] Obstacle blocking path. Awaiting clearance.")

    cmd_speeds = set_swerve_velocity(vx_body, vy_body, omega)

    last_forward_speed, last_omega = vx_body, omega

    sensor_vals = [s.getValue() for s in sensors]
    actual_wheel_vel = [(sensor_vals[i] - prev_sensor_vals[i]) / dt for i in range(4)]
    prev_sensor_vals = sensor_vals

    if DEBUG and (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        print(f"t={sim_time:.1f} | dist={dist:.2f} | unc={pos_uncertainty:.3f} | avoid_thresh={avoid_threshold_m:.2f}")
        print(f"  vx_body={vx_body:.2f} | vy_body={vy_body:.2f} | omega={omega:.2f} | min_front={min_front_range:.2f}")
        if in_crop_row:
            print(f"  [LANE] center_y={y_row:.2f} | current_y={pos[1]:.2f} | dev={y_deviation:.2f} | limit={boundary_limit_reached}")
