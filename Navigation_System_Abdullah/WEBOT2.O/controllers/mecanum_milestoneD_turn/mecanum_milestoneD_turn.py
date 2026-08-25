from controller import Robot
import math
import csv
import os
import random

# --- CONFIG (from your validated notes) ---
TIME_STEP = 32

WHEEL_RADIUS = 0.075
L = 0.3
W = 0.32
MAX_WHEEL_VEL = 20.0

REACH_THRESHOLD = 0.15
MAX_SPEED = 0.3
CRUISE_SPEED = 0.6
STRAIGHT_HEADING_THRESHOLD = math.radians(5)
K_HEADING = 2.0
OMEGA_MAX = 1.5

PRINT_INTERVAL_S = 1.0
LOG_EVERY_N_STEPS = 16

# --- Milestone C: localization noise + filter config ---
GPS_NOISE_SIGMA = 0.03
BLACKOUT_START_S = 100.0
BLACKOUT_END_S = 140.0
GPS_CORRECTION_WEIGHT = 0.4

# --- Milestone D: row-switch mode comparison ---
ROW_SWITCH_MODE = "turn"   # HARDCODED — this file is turn-mode only

DEBUG = True

# --- INIT ---
robot = Robot()

motor_names = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
motors = [robot.getDevice(name) for name in motor_names]
for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)

sensor_names = ['wheel_fl_sensor', 'wheel_fr_sensor', 'wheel_rl_sensor', 'wheel_rr_sensor']
sensors = [robot.getDevice(name) for name in sensor_names]
for s in sensors:
    s.enable(TIME_STEP)

# --- KINEMATICS ---
def set_mecanum_velocity(vx, vy, omega):
    fl = (vx - vy - (L + W) * omega) / WHEEL_RADIUS
    fr = (vx + vy + (L + W) * omega) / WHEEL_RADIUS
    rl = (vx + vy - (L + W) * omega) / WHEEL_RADIUS
    rr = (vx - vy + (L + W) * omega) / WHEEL_RADIUS

    speeds = [fl, fr, rl, rr]
    max_speed = max(abs(s) for s in speeds)
    if max_speed > MAX_WHEEL_VEL:
        scale = MAX_WHEEL_VEL / max_speed
        speeds = [s * scale for s in speeds]

    motors[0].setVelocity(speeds[0])
    motors[1].setVelocity(speeds[1])
    motors[2].setVelocity(speeds[2])
    motors[3].setVelocity(speeds[3])

    return speeds

# --- BOUSTROPHEDON PATH ---
def generate_boustrophedon(field_w=10.0, field_h=10.0, stripe_width=0.65, margin=0.5):
    waypoints = []
    x_start = -field_w / 2 + margin
    x_end = field_w / 2 - margin
    y_min = -field_h / 2 + margin
    y_max = field_h / 2 - margin

    num_stripes = int((field_h - 2 * margin) / stripe_width) + 1

    for i in range(num_stripes):
        y = min(y_min + i * stripe_width, y_max)
        if i % 2 == 0:
            waypoints.append((x_start, y))
            waypoints.append((x_end, y))
        else:
            waypoints.append((x_end, y))
            waypoints.append((x_start, y))

    return waypoints

def is_row_switch_target(idx):
    # Segment driving TOWARD waypoint idx is a row switch (pure lateral move)
    # when idx is even and not the very first target (idx==0 is the initial
    # transit from spawn, handled as a normal drive, not a stripe-to-stripe switch).
    return idx % 2 == 0 and idx > 0

waypoints = generate_boustrophedon()
current_wp_idx = 0

print(f"[Nav] Generated {len(waypoints)} waypoints")
print(f"[Nav] ROW_SWITCH_MODE = {ROW_SWITCH_MODE}")

prev_sensor_vals = [0.0, 0.0, 0.0, 0.0]
dt = TIME_STEP / 1000.0

# --- LOGGING state (Milestone B) ---
trajectory_log = []
step_count = 0

# --- PRINT THROTTLE state ---
last_print_time = -PRINT_INTERVAL_S

# --- Localization filter state (Milestone C) ---
est_x, est_y = None, None
last_forward_speed, last_omega = 0.0, 0.0
localization_log = []

# --- Row-switch instrumentation state (Milestone D) ---
last_seen_wp_idx = -1
switch_active = False
switch_start_time = 0.0
switch_start_heading = 0.0
switch_energy = 0.0
switch_count = 0
row_switch_log = []

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

            row_switch_path = f"results/row_switch_log_{ROW_SWITCH_MODE}.csv"
            with open(row_switch_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["mode", "switch_index", "start_time_s", "end_time_s",
                                  "switch_time_s", "energy_estimate"])
                writer.writerows(row_switch_log)
            print(f"[Nav] Logged {len(row_switch_log)} row-switch events to {row_switch_path}")

        set_mecanum_velocity(0, 0, 0)
        continue

    pos = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]

    tx, ty = waypoints[current_wp_idx]
    dx = tx - pos[0]
    dy = ty - pos[1]
    dist = math.hypot(dx, dy)

    sim_time = robot.getTime()

    # --- Milestone D: detect start of a new target, start switch record if row-switch ---
    if current_wp_idx != last_seen_wp_idx:
        last_seen_wp_idx = current_wp_idx
        if is_row_switch_target(current_wp_idx):
            switch_active = True
            switch_start_time = sim_time
            switch_start_heading = yaw
            switch_energy = 0.0
            switch_count += 1
        else:
            switch_active = False

    # --- trajectory logging (Milestone B) ---
    step_count += 1
    if step_count % LOG_EVERY_N_STEPS == 0:
        trajectory_log.append((sim_time, pos[0], pos[1], yaw))

    # --- localization noise injection + complementary filter (Milestone C) ---
    gt_x, gt_y = pos[0], pos[1]
    noisy_gps_x = gt_x + random.gauss(0, GPS_NOISE_SIGMA)
    noisy_gps_y = gt_y + random.gauss(0, GPS_NOISE_SIGMA)

    in_blackout = BLACKOUT_START_S <= sim_time <= BLACKOUT_END_S

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
        # --- Milestone D: finalize switch record if this was a row-switch target ---
        if switch_active:
            switch_end_time = sim_time
            row_switch_log.append((
                ROW_SWITCH_MODE, switch_count, switch_start_time, switch_end_time,
                switch_end_time - switch_start_time, switch_energy
            ))
            switch_active = False

        current_wp_idx += 1
        print(f"[Nav] Reached waypoint {current_wp_idx}/{len(waypoints)}")
        continue

    # --- Navigation: existing heading-correction controller for ALL segments,
    #     including row-switches. No crab logic in this file — a row-switch
    #     target here naturally produces a turn-then-drive maneuver since the
    #     target is directly sideways (dx=0).
    target_heading = math.atan2(dy, dx)
    heading_error = target_heading - yaw
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_error))
    speed_cap = CRUISE_SPEED if abs(heading_error) < STRAIGHT_HEADING_THRESHOLD else MAX_SPEED
    forward_speed = min(speed_cap, dist) * max(0.0, math.cos(heading_error))

    cmd_speeds = set_mecanum_velocity(forward_speed, 0.0, omega)

    last_forward_speed, last_omega = forward_speed, omega

    # --- Milestone D: accumulate energy proxy during active switch window ---
    if switch_active:
        switch_energy += sum(abs(s) for s in cmd_speeds) * dt

    sensor_vals = [s.getValue() for s in sensors]
    actual_wheel_vel = [(sensor_vals[i] - prev_sensor_vals[i]) / dt for i in range(4)]
    prev_sensor_vals = sensor_vals

    if DEBUG and (sim_time - last_print_time) >= PRINT_INTERVAL_S:
        last_print_time = sim_time
        print(f"t={sim_time:.1f} dist={dist:.2f} heading_err={math.degrees(heading_error):.1f} "
              f"fwd={forward_speed:.2f} omega={omega:.2f} switch_active={switch_active}")
        print(f"  cmd_wheel_vel=[{cmd_speeds[0]:.1f}, {cmd_speeds[1]:.1f}, "
              f"{cmd_speeds[2]:.1f}, {cmd_speeds[3]:.1f}]  "
              f"actual_wheel_vel=[{actual_wheel_vel[0]:.1f}, {actual_wheel_vel[1]:.1f}, "
              f"{actual_wheel_vel[2]:.1f}, {actual_wheel_vel[3]:.1f}]")
