import sys
import os
import math
import csv
import numpy as np
from controller import Robot

# Add parent workspace to sys.path so we can import config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "Delta-Robot-Subsystem")))

import config as cfg

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
COORDINATION_MODE = os.environ.get("COORDINATION_MODE", "DENSITY_ADAPTIVE_SPEED")

# --- INIT WEBOTS ---
robot = Robot()
print(f"[ArmCoord] Starting controller in mode: {COORDINATION_MODE}")

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

# Serial Arm Rotational Motors
arm_motor_1 = robot.getDevice("arm_motor_1") # base yaw (Z)
arm_motor_2 = robot.getDevice("arm_motor_2") # shoulder pitch (Y)
arm_motor_3 = robot.getDevice("arm_motor_3") # elbow pitch (Y)

arm_sens_1 = robot.getDevice("arm_sensor_1")
arm_sens_1.enable(TIME_STEP)
arm_sens_2 = robot.getDevice("arm_sensor_2")
arm_sens_2.enable(TIME_STEP)
arm_sens_3 = robot.getDevice("arm_sensor_3")
arm_sens_3.enable(TIME_STEP)

# Set Serial Arm limits & velocities (matching servo speeds)
# 2.0 rad/s (~114 deg/s) to mimic loaded performance
arm_motor_1.setVelocity(2.0)
arm_motor_2.setVelocity(2.0)
arm_motor_3.setVelocity(2.0)

# --- SERIAL LINK KINEMATICS ---
class SerialArmKinematics:
    def __init__(self):
        self.L1 = 120.0 # mm (upper arm length)
        self.L2 = 230.0 # mm (forearm length)
        self.z_offset = -70.0 # mm (shoulder vertical coordinate)

    def inverse(self, x, y, z):
        """
        Target (x, y, z) in robot frame (mm) -> 3 joint angles in radians.
        Returns (theta1, theta2, theta3). None if unreachable.
        """
        # Yaw angle
        t1 = math.atan2(y, x)
        
        # Project onto vertical plane
        r = math.hypot(x, y)
        z_rel = z - self.z_offset
        
        # Planar 2-link IK
        # kinematic equations mapping to straight-down zero position:
        # r = L1 * sin(t2) + L2 * sin(t2 + t3)
        # z_rel = -L1 * cos(t2) - L2 * cos(t2 + t3)
        # Solve using cos law:
        cos_t3 = (r**2 + z_rel**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if cos_t3 < -1.0 or cos_t3 > 1.0:
            return None, None, None
            
        t3 = math.acos(cos_t3) # elbow down configuration
        
        t2 = math.atan2(r, -z_rel) - math.atan2(self.L2 * math.sin(t3), self.L1 + self.L2 * math.cos(t3))
        
        return t1, t2, t3

    def forward(self, t1, t2, t3):
        if t1 is None or t2 is None or t3 is None:
            return None
        r = self.L1 * math.sin(t2) + self.L2 * math.sin(t2 + t3)
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        z = -self.L1 * math.cos(t2) - self.L2 * math.cos(t2 + t3) + self.z_offset
        return x, y, z

arm_kin = SerialArmKinematics()

# --- DETERMINISTIC WEED FIELD GENERATION ---
def generate_field_weeds(seed=42):
    rng = np.random.default_rng(seed)
    weeds = []
    stripes_y = np.arange(-4.0, 4.0, 0.65)
    for y in stripes_y:
        n = rng.integers(3, 7)
        for _ in range(n):
            wx = rng.uniform(-4.0, 4.0)
            wy = y + rng.uniform(-0.15, 0.15)
            wz = 0.0 # Ground height
            weeds.append((wx, wy, wz))
    return weeds

global_weeds = generate_field_weeds()
print(f"[ArmCoord] Spawned {len(global_weeds)} weeds across the field.")

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
weed_queue = [] # list of (wx, wy, idx) in global frame
processed_weeds = set()
cut_count = 0
missed_count = 0

# Arm state: "HOME", "APPROACH", "PLUNGE", "RETRACT"
arm_state = "HOME"
arm_timer = 0.0
active_target_global = None # (wx, wy, idx)

# Camera detection zone (lookahead)
CAM_LOOKAHEAD_X = 0.35
CAM_WIDTH_Y = 0.35
CAM_LENGTH_X = 0.25

# Target operating limits
ROBOT_Z_CLEARANCE = -50.0  # mm
ROBOT_Z_CUT = -170.0       # mm

# Results logging
pid_log = [] # time, target_z, actual_z, theta1, theta2, theta3

# Home joint angles (straight down, slightly retracted)
# We choose a default pose where it is retracted
home_t1, home_t2, home_t3 = 0.0, 0.0, 0.0

# Set initial arm position
arm_motor_1.setPosition(home_t1)
arm_motor_2.setPosition(home_t2)
arm_motor_3.setPosition(home_t3)

dt = TIME_STEP / 1000.0
sim_time = 0.0

while robot.step(TIME_STEP) != -1:
    sim_time = robot.getTime()
    
    # Get current pose
    pos = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    
    # Check if navigation is done
    if current_wp_idx >= len(waypoints):
        set_mecanum_velocity(0, 0, 0)
        print(f"[ArmCoord] Navigation finished! Cut: {cut_count}, Missed: {missed_count}")
        
        # Save results
        os.makedirs("results", exist_ok=True)
        results_file = f"results/summary_arm_{COORDINATION_MODE}.csv"
        with open(results_file, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            w.writerow(["mode", COORDINATION_MODE])
            w.writerow(["cut_weeds", cut_count])
            w.writerow(["missed_weeds", missed_count])
            w.writerow(["total_weeds", len(global_weeds)])
            w.writerow(["efficiency_pct", round(100 * cut_count / (cut_count + missed_count + 1e-9), 2)])
            w.writerow(["sim_time_s", round(sim_time, 2)])
        print(f"[ArmCoord] Saved results summary to {results_file}")
        
        # Save PID step response log
        with open(f"results/pid_log_arm_{COORDINATION_MODE}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "target_z_mm", "actual_z_mm", "theta1", "theta2", "theta3"])
            w.writerows(pid_log)
        break
        
    # --- CAMERA WEED DETECTION ---
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    for idx, (wx, wy, wz) in enumerate(global_weeds):
        if idx in processed_weeds:
            continue
        
        dx_g = wx - pos[0]
        dy_g = wy - pos[1]
        
        lx = dx_g * cos_y + dy_g * sin_y
        ly = -dx_g * sin_y + dy_g * cos_y
        
        if (CAM_LOOKAHEAD_X <= lx <= CAM_LOOKAHEAD_X + CAM_LENGTH_X) and (-CAM_WIDTH_Y <= ly <= CAM_WIDTH_Y):
            processed_weeds.add(idx)
            weed_queue.append((wx, wy, idx))
            print(f"[ArmCoord] Detected weed {idx} at local x={lx:.2f}, y={ly:.2f}")

    # --- PATH OPTIMIZATION (SORTING) ---
    if len(weed_queue) > 1 and arm_state == "HOME":
        sorted_queue = []
        curr_loc = np.array([0.0, 0.0])
        unvisited = list(weed_queue)
        while unvisited:
            local_pts = []
            for wx, wy, w_idx in unvisited:
                dx_g = wx - pos[0]
                dy_g = wy - pos[1]
                lx = dx_g * cos_y + dy_g * sin_y
                ly = -dx_g * sin_y + dy_g * cos_y
                local_pts.append((lx, ly, wx, wy, w_idx))
            
            closest = min(local_pts, key=lambda p: np.hypot(p[0] - curr_loc[0], p[1] - curr_loc[1]))
            sorted_queue.append((closest[2], closest[3], closest[4]))
            for item in unvisited:
                if item[2] == closest[4]:
                    unvisited.remove(item)
                    break
            curr_loc = np.array([closest[0], closest[1]])
        weed_queue = sorted_queue

    # --- DASC ALGORITHM SPEED CONTROLLER ---
    target_wp = waypoints[current_wp_idx]
    nav_dx = target_wp[0] - pos[0]
    nav_dy = target_wp[1] - pos[1]
    nav_dist = math.hypot(nav_dx, nav_dy)
    
    if nav_dist < REACH_THRESHOLD:
        current_wp_idx += 1
        print(f"[ArmCoord] Reached waypoint {current_wp_idx}/{len(waypoints)}")
        continue
        
    target_heading = math.atan2(nav_dy, nav_dx)
    heading_err = target_heading - yaw
    heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
    
    base_omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_err))
    
    base_fwd_speed = 0.0
    if COORDINATION_MODE == "STOP_AND_CUT":
        if len(weed_queue) > 0 or arm_state != "HOME":
            base_fwd_speed = 0.0
            base_omega = 0.0
        else:
            base_fwd_speed = CRUISE_SPEED if abs(heading_err) < 0.1 else MAX_SPEED
    elif COORDINATION_MODE == "CONTINUOUS_TRACKING":
        base_fwd_speed = CRUISE_SPEED if abs(heading_err) < 0.1 else MAX_SPEED
    elif COORDINATION_MODE == "DENSITY_ADAPTIVE_SPEED":
        q_len = len(weed_queue)
        if arm_state != "HOME":
            q_len += 1
        speed_factor = max(0.0, 1.0 - q_len / 3.0)
        base_fwd_speed = CRUISE_SPEED * speed_factor if abs(heading_err) < 0.1 else MAX_SPEED * speed_factor
    
    base_fwd_speed = base_fwd_speed * max(0.0, math.cos(heading_err))
    set_mecanum_velocity(base_fwd_speed, 0.0, base_omega)

    # --- SERIAL ARM COORDINATION STATE MACHINE ---
    if arm_state == "HOME":
        if len(weed_queue) > 0:
            active_target_global = weed_queue.pop(0)
            arm_state = "APPROACH"
            arm_timer = 0.0
            print(f"[ArmCoord] Starting excision for weed {active_target_global[2]}")
        else:
            # Command default home pose
            arm_motor_1.setPosition(home_t1)
            arm_motor_2.setPosition(home_t2)
            arm_motor_3.setPosition(home_t3)
            
    if arm_state != "HOME":
        # Compute dynamic local coordinates of target weed
        wx, wy, w_idx = active_target_global
        dx_g = wx - pos[0]
        dy_g = wy - pos[1]
        lx = dx_g * cos_y + dy_g * sin_y
        ly = -dx_g * sin_y + dy_g * cos_y
        
        # Horizontal distance checks (serial arm range max is L1 + L2 = 350mm)
        local_dist = math.hypot(lx, ly)
        if local_dist > 0.33: # serial arm workspace margin
            print(f"[ArmCoord] Weed {w_idx} slipped out of arm workspace! Missed.")
            missed_count += 1
            arm_state = "HOME"
            active_target_global = None
            continue
            
        target_x_mm = lx * 1000.0
        target_y_mm = ly * 1000.0
        
        # State transitions
        if arm_state == "APPROACH":
            # Command inverse kinematics joint angles at clearance height
            t1, t2, t3 = arm_kin.inverse(target_x_mm, target_y_mm, ROBOT_Z_CLEARANCE)
            if t1 is not None:
                arm_motor_1.setPosition(t1)
                arm_motor_2.setPosition(t2)
                arm_motor_3.setPosition(t3)
            
            # Check sensor alignment
            a1, a2, a3 = arm_sens_1.getValue(), arm_sens_2.getValue(), arm_sens_3.getValue()
            if t1 is not None and math.hypot(a1 - t1, a2 - t2) < 0.05 or arm_timer > 1.2:
                arm_state = "PLUNGE"
                arm_timer = 0.0
                print(f"[ArmCoord] Plunging on weed {w_idx}")
                
        elif arm_state == "PLUNGE":
            t1, t2, t3 = arm_kin.inverse(target_x_mm, target_y_mm, ROBOT_Z_CUT)
            if t1 is not None:
                arm_motor_1.setPosition(t1)
                arm_motor_2.setPosition(t2)
                arm_motor_3.setPosition(t3)
            
            a3 = arm_sens_3.getValue()
            if t3 is not None and abs(a3 - t3) < 0.05 or arm_timer > 1.2:
                arm_state = "RETRACT"
                arm_timer = 0.0
                cut_count += 1
                print(f"[ArmCoord] Cut weed {w_idx}! Retracting.")
                
        elif arm_state == "RETRACT":
            t1, t2, t3 = arm_kin.inverse(target_x_mm, target_y_mm, ROBOT_Z_CLEARANCE)
            if t1 is not None:
                arm_motor_1.setPosition(t1)
                arm_motor_2.setPosition(t2)
                arm_motor_3.setPosition(t3)
            
            a2, a3 = arm_sens_2.getValue(), arm_sens_3.getValue()
            if t2 is not None and math.hypot(a2 - t2, a3 - t3) < 0.05 or arm_timer > 1.2:
                arm_state = "HOME"
                active_target_global = None
                print(f"[ArmCoord] Retraction finished.")
                
        arm_timer += dt
        
        # Log angles and target heights
        a1, a2, a3 = arm_sens_1.getValue(), arm_sens_2.getValue(), arm_sens_3.getValue()
        curr_xyz = arm_kin.forward(a1, a2, a3)
        actual_z = curr_xyz[2] if curr_xyz else 0.0
        tz_mm = ROBOT_Z_CUT if arm_state == "PLUNGE" else ROBOT_Z_CLEARANCE
        pid_log.append((sim_time, tz_mm, actual_z, math.degrees(a1), math.degrees(a2), math.degrees(a3)))
