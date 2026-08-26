import sys
import os
import math
import csv
import numpy as np
from controller import Robot

# Add parent workspace to sys.path so we can import delta kinematics and config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "Delta-Robot-Subsystem")))

import config as cfg
from delta_robot_kinematics import DeltaKinematics

# --- SIMULATION CONFIG ---
TIME_STEP = 32
WHEEL_RADIUS = 0.075
L = 0.3
W = 0.32
MAX_WHEEL_VEL = 20.0

REACH_THRESHOLD = 0.15
CRUISE_SPEED = 0.4
MAX_SPEED = 0.3
K_HEADING = 2.0
OMEGA_MAX = 1.5

# Coordination Modes: "STOP_AND_CUT", "CONTINUOUS_TRACKING", "DENSITY_ADAPTIVE_SPEED"
# We can read this from an environment variable or default to DENSITY_ADAPTIVE_SPEED
COORDINATION_MODE = os.environ.get("COORDINATION_MODE", "DENSITY_ADAPTIVE_SPEED")

# --- INIT WEBOTS ---
robot = Robot()
print(f"[DeltaCoord] Starting controller in mode: {COORDINATION_MODE}")

# Wheel motors
motor_names = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
wheels = [robot.getDevice(name) for name in motor_names]
for w in wheels:
    w.setPosition(float('inf'))
    w.setVelocity(0.0)

# Sensors
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu = robot.getDevice('imu')
imu.enable(TIME_STEP)

# Delta Robot Linear Motors (Option 1 Slider representation)
delta_x = robot.getDevice("delta_x_motor")
delta_y = robot.getDevice("delta_y_motor")
delta_z = robot.getDevice("delta_z_motor")

delta_x_sens = robot.getDevice("delta_x_sensor")
delta_x_sens.enable(TIME_STEP)
delta_y_sens = robot.getDevice("delta_y_sensor")
delta_y_sens.enable(TIME_STEP)
delta_z_sens = robot.getDevice("delta_z_sensor")
delta_z_sens.enable(TIME_STEP)

# Set Delta Motors limits & velocities
delta_x.setVelocity(0.5)
delta_y.setVelocity(0.5)
delta_z.setVelocity(0.5)

# Kinematics solver
kin = DeltaKinematics()

# --- DETERMINISTIC WEED FIELD GENERATION ---
# Let's generate a reproducible set of weeds on the 10x10 field
def generate_field_weeds(seed=42):
    rng = np.random.default_rng(seed)
    weeds = []
    # Generate weeds in stripes along the boustrophedon path
    # Path has Y stripes from -4.5 to 4.5 in steps of 0.65
    stripes_y = np.arange(-4.0, 4.0, 0.65)
    for y in stripes_y:
        # Place 4-6 weeds per stripe
        n = rng.integers(3, 7)
        for _ in range(n):
            wx = rng.uniform(-4.0, 4.0)
            wy = y + rng.uniform(-0.15, 0.15)
            wz = 0.0 # Ground height
            weeds.append((wx, wy, wz))
    return weeds

global_weeds = generate_field_weeds()
print(f"[DeltaCoord] Spawned {len(global_weeds)} weeds across the field.")

# --- NAVIGATION PATH ---
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

# --- CONTROL VARIABLES ---
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
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
    return speeds

# --- WEEDING EXECUTION QUEUE ---
weed_queue = [] # list of (wx, wy) in global frame
processed_weeds = set()
cut_count = 0
missed_count = 0

# Arm state: "HOME", "APPROACH", "PLUNGE", "RETRACT"
arm_state = "HOME"
arm_timer = 0.0
active_target_local = None # (x, y, z) in robot frame
active_target_global = None # (x, y)

# Camera detection zone (lookahead)
CAM_LOOKAHEAD_X = 0.35 # meters ahead of robot center
CAM_WIDTH_Y = 0.35     # half width
CAM_LENGTH_X = 0.25    # size of detection box

# Target operating limits
ROBOT_Z_CLEARANCE = -50.0  # mm
ROBOT_Z_CUT = -170.0       # mm

# Results logging
metrics_log = []
pid_log = [] # time, target_z, actual_z, theta1, theta2, theta3

# --- MAIN LOOP ---
dt = TIME_STEP / 1000.0
sim_time = 0.0

# Set initial arm position
delta_x.setPosition(0.0)
delta_y.setPosition(0.0)
delta_z.setPosition(ROBOT_Z_CLEARANCE / 1000.0)

while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()
    
    # Get current pose
    pos = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    
    # Check if navigation is done
    if current_wp_idx >= len(waypoints):
        set_mecanum_velocity(0, 0, 0)
        print(f"[DeltaCoord] Navigation finished! Cut: {cut_count}, Missed: {missed_count}")
        
        # Save results
        os.makedirs("results", exist_ok=True)
        results_file = f"results/summary_{COORDINATION_MODE}.csv"
        with open(results_file, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            w.writerow(["mode", COORDINATION_MODE])
            w.writerow(["cut_weeds", cut_count])
            w.writerow(["missed_weeds", missed_count])
            w.writerow(["total_weeds", len(global_weeds)])
            w.writerow(["efficiency_pct", round(100 * cut_count / (cut_count + missed_count + 1e-9), 2)])
            w.writerow(["sim_time_s", round(sim_time, 2)])
        print(f"[DeltaCoord] Saved results summary to {results_file}")
        
        # Save PID step response log
        with open(f"results/pid_log_{COORDINATION_MODE}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "target_z_mm", "actual_z_mm", "theta1", "theta2", "theta3"])
            w.writerows(pid_log)
        break
        
    # --- CAMERA WEED DETECTION ---
    # Detect weeds that fall into the lookahead camera frame
    # Camera frame is offset from robot center along heading
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    for idx, (wx, wy, wz) in enumerate(global_weeds):
        if idx in processed_weeds:
            continue
        
        # Transform weed to robot-centric coordinates
        dx_g = wx - pos[0]
        dy_g = wy - pos[1]
        
        # Robot local coordinates: x is forward, y is left
        lx = dx_g * cos_y + dy_g * sin_y
        ly = -dx_g * sin_y + dy_g * cos_y
        
        # Camera lookahead zone check
        # We check if weed is inside the rectangular camera patch in front of the base
        if (CAM_LOOKAHEAD_X <= lx <= CAM_LOOKAHEAD_X + CAM_LENGTH_X) and (-CAM_WIDTH_Y <= ly <= CAM_WIDTH_Y):
            processed_weeds.add(idx)
            weed_queue.append((wx, wy, idx))
            print(f"[DeltaCoord] Detected weed {idx} at local x={lx:.2f}, y={ly:.2f}")

    # --- PATH OPTIMIZATION (SORTING) ---
    # If the queue has elements and we are not currently weeding, sort them by distance to current robot center
    if len(weed_queue) > 1 and arm_state == "HOME":
        # Greedy Nearest Neighbor Sort
        sorted_queue = []
        curr_loc = np.array([0.0, 0.0]) # delta arm is mounted at center (0,0)
        unvisited = list(weed_queue)
        while unvisited:
            # Calculate local positions of unvisited weeds
            local_pts = []
            for wx, wy, w_idx in unvisited:
                dx_g = wx - pos[0]
                dy_g = wy - pos[1]
                lx = dx_g * cos_y + dy_g * sin_y
                ly = -dx_g * sin_y + dy_g * cos_y
                local_pts.append((lx, ly, wx, wy, w_idx))
            
            # Find closest to curr_loc
            closest = min(local_pts, key=lambda p: np.hypot(p[0] - curr_loc[0], p[1] - curr_loc[1]))
            sorted_queue.append((closest[2], closest[3], closest[4]))
            # Remove from unvisited
            for item in unvisited:
                if item[2] == closest[4]:
                    unvisited.remove(item)
                    break
            curr_loc = np.array([closest[0], closest[1]])
        weed_queue = sorted_queue

    # --- DASC ALGORITHM SPEED CONTROLLER ---
    # Calculate base velocity command
    target_wp = waypoints[current_wp_idx]
    nav_dx = target_wp[0] - pos[0]
    nav_dy = target_wp[1] - pos[1]
    nav_dist = math.hypot(nav_dx, nav_dy)
    
    if nav_dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[DeltaCoord] Reached waypoint {current_wp_idx}/{len(waypoints)}")
        continue
        
    target_heading = math.atan2(nav_dy, nav_dx)
    heading_err = target_heading - yaw
    heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
    
    base_omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_err))
    
    # Adjust base forward speed based on coordination mode
    base_fwd_speed = 0.0
    if COORDINATION_MODE == "STOP_AND_CUT":
        if len(weed_queue) > 0 or arm_state != "HOME":
            base_fwd_speed = 0.0
            base_omega = 0.0 # Stop completely
        else:
            base_fwd_speed = CRUISE_SPEED if abs(heading_err) < 0.1 else MAX_SPEED
    elif COORDINATION_MODE == "CONTINUOUS_TRACKING":
        # Keep driving forward continuously
        base_fwd_speed = CRUISE_SPEED if abs(heading_err) < 0.1 else MAX_SPEED
    elif COORDINATION_MODE == "DENSITY_ADAPTIVE_SPEED":
        # Dynamic speed scaling
        q_len = len(weed_queue)
        if arm_state != "HOME":
            q_len += 1
        speed_factor = max(0.0, 1.0 - q_len / 3.0)
        base_fwd_speed = CRUISE_SPEED * speed_factor if abs(heading_err) < 0.1 else MAX_SPEED * speed_factor
    
    # Apply heading correction taper
    base_fwd_speed = base_fwd_speed * max(0.0, math.cos(heading_err))
    set_mecanum_velocity(base_fwd_speed, 0.0, base_omega)

    # --- DELTA ROBOT COORDINATION STATE MACHINE ---
    if arm_state == "HOME":
        if len(weed_queue) > 0:
            active_target_global = weed_queue.pop(0)
            arm_state = "APPROACH"
            arm_timer = 0.0
            print(f"[DeltaCoord] Starting excision for weed {active_target_global[2]}")
        else:
            # Hold home position
            delta_x.setPosition(0.0)
            delta_y.setPosition(0.0)
            delta_z.setPosition(ROBOT_Z_CLEARANCE / 1000.0)
            
    if arm_state != "HOME":
        # Compute dynamic local coordinates of the target weed
        wx, wy, w_idx = active_target_global
        dx_g = wx - pos[0]
        dy_g = wy - pos[1]
        lx = dx_g * cos_y + dy_g * sin_y
        ly = -dx_g * sin_y + dy_g * cos_y
        
        # Check if the weed has slipped past the posterior operating limit
        # Delta workspace horizontal radius is ~200mm (0.2m)
        local_dist = math.hypot(lx, ly)
        if local_dist > 0.23: # weed slipped out of physical limits
            print(f"[DeltaCoord] Weed {w_idx} slipped out of workspace! Missed.")
            missed_count += 1
            arm_state = "HOME"
            active_target_global = None
            delta_z.setPosition(ROBOT_Z_CLEARANCE / 1000.0)
            continue
            
        # Target coordinate in mm
        target_x_mm = lx * 1000.0
        target_y_mm = ly * 1000.0
        
        # State transitions
        if arm_state == "APPROACH":
            # Command horizontal alignment, hold clearance height
            delta_x.setPosition(lx)
            delta_y.setPosition(ly)
            delta_z.setPosition(ROBOT_Z_CLEARANCE / 1000.0)
            
            # Check if aligned (sensor readings within 2mm)
            ax = delta_x_sens.getValue()
            ay = delta_y_sens.getValue()
            if math.hypot(ax - lx, ay - ly) < 0.005 or arm_timer > 0.8:
                arm_state = "PLUNGE"
                arm_timer = 0.0
                print(f"[DeltaCoord] Plunging on weed {w_idx}")
                
        elif arm_state == "PLUNGE":
            # Hold horizontal alignment, command plunge depth
            delta_x.setPosition(lx)
            delta_y.setPosition(ly)
            delta_z.setPosition(ROBOT_Z_CUT / 1000.0)
            
            # Check if reached cut depth
            az = delta_z_sens.getValue()
            if abs(az - (ROBOT_Z_CUT / 1000.0)) < 0.005 or arm_timer > 0.8:
                # Excision complete!
                arm_state = "RETRACT"
                arm_timer = 0.0
                cut_count += 1
                print(f"[DeltaCoord] Cut weed {w_idx}! Retracting.")
                
        elif arm_state == "RETRACT":
            # Hold horizontal alignment, command clearance height
            delta_x.setPosition(lx)
            delta_y.setPosition(ly)
            delta_z.setPosition(ROBOT_Z_CLEARANCE / 1000.0)
            
            # Check if back to clearance
            az = delta_z_sens.getValue()
            if abs(az - (ROBOT_Z_CLEARANCE / 1000.0)) < 0.005 or arm_timer > 0.8:
                arm_state = "HOME"
                active_target_global = None
                print(f"[DeltaCoord] Retraction finished.")
                
        arm_timer += dt
        
        # Log kinematics and PID response
        actual_x_mm = delta_x_sens.getValue() * 1000.0
        actual_y_mm = delta_y_sens.getValue() * 1000.0
        actual_z_mm = delta_z_sens.getValue() * 1000.0
        
        # Target depth
        tz_mm = ROBOT_Z_CUT if arm_state == "PLUNGE" else ROBOT_Z_CLEARANCE
        
        # Solve inverse kinematics to calculate joint angles for logging
        angles = kin.inverse(actual_x_mm, actual_y_mm, actual_z_mm, enforce_limits=False)
        theta1 = angles[0] if angles[0] is not None else 0.0
        theta2 = angles[1] if angles[1] is not None else 0.0
        theta3 = angles[2] if angles[2] is not None else 0.0
        
        pid_log.append((sim_time, tz_mm, actual_z_mm, theta1, theta2, theta3))
