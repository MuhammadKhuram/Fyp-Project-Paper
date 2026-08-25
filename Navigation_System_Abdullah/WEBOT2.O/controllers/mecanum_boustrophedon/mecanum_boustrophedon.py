##Latest Code##
from controller import Robot
import math

# --- CONFIG (from your validated notes) ---
TIME_STEP = 32

WHEEL_RADIUS = 0.075
L = 0.3
W = 0.32
MAX_WHEEL_VEL = 20.0

REACH_THRESHOLD = 0.15
MAX_SPEED = 0.3
K_HEADING = 2.0
OMEGA_MAX = 1.5   # validated fix, confirmed working from last test run

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

# Wheel position sensors — needed to see ACTUAL wheel speed, since
# motor.getVelocity() only returns the commanded setpoint, not reality.
sensor_names = ['wheel_fl_sensor', 'wheel_fr_sensor', 'wheel_rl_sensor', 'wheel_rr_sensor']
sensors = [robot.getDevice(name) for name in sensor_names]
for s in sensors:
    s.enable(TIME_STEP)

# --- KINEMATICS (your real physics-based equations, unchanged) ---
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

waypoints = generate_boustrophedon()
current_wp_idx = 0

print(f"[Nav] Generated {len(waypoints)} waypoints")

# PositionSensor gives cumulative wheel angle (rad), not velocity — so we
# differentiate it ourselves: (this_reading - last_reading) / dt = rad/s
prev_sensor_vals = [0.0, 0.0, 0.0, 0.0]
dt = TIME_STEP / 1000.0

# --- MAIN LOOP ---
while robot.step(TIME_STEP) != -1:
    if current_wp_idx >= len(waypoints):
        if current_wp_idx == len(waypoints):  # print exactly once
            print("[Nav] ALL WAYPOINTS COMPLETE")
            current_wp_idx += 1  # step past so this block doesn't reprint
        set_mecanum_velocity(0, 0, 0)
        continue

    pos = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]

    tx, ty = waypoints[current_wp_idx]
    dx = tx - pos[0]
    dy = ty - pos[1]
    dist = math.hypot(dx, dy)

    if dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[Nav] Reached waypoint {current_wp_idx}/{len(waypoints)}")
        continue

    target_heading = math.atan2(dy, dx)   # confirmed convention from your notes
    heading_error = target_heading - yaw
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # wrap to [-pi, pi]

    # --- validated fix ---
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_error))
    forward_speed = min(MAX_SPEED, dist) * max(0.0, math.cos(heading_error))

    cmd_speeds = set_mecanum_velocity(forward_speed, 0.0, omega)

    sensor_vals = [s.getValue() for s in sensors]
    actual_wheel_vel = [(sensor_vals[i] - prev_sensor_vals[i]) / dt for i in range(4)]
    prev_sensor_vals = sensor_vals

    if DEBUG:
        print(f"dist={dist:.2f} heading_err={math.degrees(heading_error):.1f} "
              f"fwd={forward_speed:.2f} omega={omega:.2f}")
        print(f"  cmd_wheel_vel=[{cmd_speeds[0]:.1f}, {cmd_speeds[1]:.1f}, "
              f"{cmd_speeds[2]:.1f}, {cmd_speeds[3]:.1f}]  "
              f"actual_wheel_vel=[{actual_wheel_vel[0]:.1f}, {actual_wheel_vel[1]:.1f}, "
              f"{actual_wheel_vel[2]:.1f}, {actual_wheel_vel[3]:.1f}]")
