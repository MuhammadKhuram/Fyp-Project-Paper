import os
import math
import csv
import numpy as np
import matplotlib.pyplot as plt

# --- GEOMETRY & LIMITS CONFIG ---
L1 = 120.0 # mm (upper arm)
L2 = 230.0 # mm (forearm)
Z_CLEARANCE = -50.0 # mm
Z_CUT = -170.0 # mm
ARM_MAX_VEL_MM_S = 500.0 # 500 mm/s (0.5 m/s)
JOINT_MAX_VEL_RAD_S = 2.0 # 2.0 rad/s (~114 deg/s)

MAX_SPEED = 0.3
OMEGA_MAX = 1.5
K_HEADING = 2.0
REACH_THRESHOLD = 0.15 # meters

FIELD_SIZE = 10.0 # meters
STRIPE_WIDTH = 0.65
MARGIN = 0.5

# --- BOUSTROPHEDON WAYPOINTS ---
def generate_boustrophedon():
    waypoints = []
    x_start = -FIELD_SIZE / 2 + MARGIN
    x_end = FIELD_SIZE / 2 - MARGIN
    y_min = -FIELD_SIZE / 2 + MARGIN
    y_max = FIELD_SIZE / 2 - MARGIN
    num_stripes = int((FIELD_SIZE - 2 * MARGIN) / STRIPE_WIDTH) + 1
    for i in range(num_stripes):
        y = min(y_min + i * STRIPE_WIDTH, y_max)
        if i % 2 == 0:
            waypoints.append((x_start, y))
            waypoints.append((x_end, y))
        else:
            waypoints.append((x_end, y))
            waypoints.append((x_start, y))
    return waypoints

# --- DETERMINISTIC WEED FIELD GENERATION ---
def generate_weeds(seed=42):
    rng = np.random.default_rng(seed)
    weeds = []
    waypoints = generate_boustrophedon()
    stripes_y = sorted(list(set(wp[1] for wp in waypoints)))
    for idx, y in enumerate(stripes_y):
        # We generate a realistic density: 4-6 weeds per stripe segment
        n = rng.integers(3, 7)
        for _ in range(n):
            wx = rng.uniform(-4.0, 4.0)
            wy = y + rng.uniform(-0.02, 0.02) # small offset to stay in crop row
            weeds.append((wx, wy, idx))
    return weeds

# --- KINEMATICS ---
class SerialArmKinematics:
    def __init__(self):
        self.L1 = L1
        self.L2 = L2
        self.z_offset = -70.0 # mm

    def inverse(self, x, y, z):
        t1 = math.atan2(y, x)
        r = math.hypot(x, y)
        z_rel = z - self.z_offset
        cos_t3 = (r**2 + z_rel**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if cos_t3 < -1.0 or cos_t3 > 1.0:
            return None, None, None
        t3 = math.acos(cos_t3)
        t2 = math.atan2(r, -z_rel) - math.atan2(self.L2 * math.sin(t3), self.L1 + self.L2 * math.cos(t3))
        return t1, t2, t3

    def forward(self, t1, t2, t3):
        if t1 is None: return None
        r = self.L1 * math.sin(t2) + self.L2 * math.sin(t2 + t3)
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        z = -self.L1 * math.cos(t2) - self.L2 * math.cos(t2 + t3) + self.z_offset
        return x, y, z

serial_kin = SerialArmKinematics()

# Helper IK solver for delta arm
def kin_solve_delta(x, y, z):
    sb, sp = 150.0, 50.0
    L, l = 120.0, 230.0
    wb = (np.sqrt(3) / 6) * sb
    wp = (np.sqrt(3) / 6) * sp
    a = wb - wp
    
    def _solve_leg(x_l, y_l, z_l):
        E = 2 * L * (y_l + a)
        F = 2 * z_l * L
        G = x_l**2 + y_l**2 + z_l**2 + a**2 + L**2 + 2 * y_l * a - l**2
        dist = E**2 + F**2
        if G**2 > dist:
            return 0.0
        t = (-F - np.sqrt(dist - G**2)) / (G - E)
        return np.degrees(2 * np.arctan(t))
        
    c120, s120 = -0.5, np.sqrt(3) / 2
    t1 = _solve_leg(x, y, z)
    t2 = _solve_leg(x * c120 + y * s120, x * -s120 + y * c120, z)
    t3 = _solve_leg(x * c120 - y * s120, x * s120 + y * c120, z)
    return t1, t2, t3

# --- SIMULATE EXPERIMENT ---
def run_simulation(arm_type="delta", coordination_mode="DENSITY_ADAPTIVE_SPEED", cruise_speed=0.4):
    waypoints = generate_boustrophedon()
    weeds = generate_weeds()
    
    # Platform pose: x, y, theta
    px, py, pt = -4.5, -4.5, 0.0
    current_wp_idx = 0
    
    # Simulation timing
    dt = 0.032 # 32ms physics steps
    t = 0.0
    
    # Weeding state
    weed_queue = []
    processed_weeds = set()
    cut_count = 0
    missed_count = 0
    
    # Arm state: "HOME", "APPROACH", "PLUNGE", "RETRACT"
    arm_state = "HOME"
    arm_timer = 0.0
    active_target_global = None
    
    # Delta arm position state (meters relative to robot center)
    delta_x, delta_y, delta_z = 0.0, 0.0, Z_CLEARANCE / 1000.0
    
    # Serial arm angles state (radians)
    s1, s2, s3 = 0.0, 0.0, 0.0
    
    # Energy variables
    wheel_energy = 0.0
    arm_energy = 0.0
    
    # Work limit
    reach_limit = 0.20 if arm_type == "delta" else 0.30 # m
    
    while current_wp_idx < len(waypoints) and t < 600.0:
        cos_pt = math.cos(pt)
        sin_pt = math.sin(pt)
        
        # 1. Camera detection check
        for w_idx, (wx, wy, w_id) in enumerate(weeds):
            if w_idx in processed_weeds:
                continue
            
            # Local coordinates
            dx_g = wx - px
            dy_g = wy - py
            lx = dx_g * cos_pt + dy_g * sin_pt
            ly = -dx_g * sin_pt + dy_g * cos_pt
            
            # Camera lookahead check (350mm to 600mm in front of robot)
            if (0.35 <= lx <= 0.60) and (-0.35 <= ly <= 0.35):
                processed_weeds.add(w_idx)
                weed_queue.append((wx, wy, w_idx))
        
        # 2. Monitor Queue for Misses
        active_queue = []
        for wx, wy, w_idx in weed_queue:
            dx_g = wx - px
            dy_g = wy - py
            lx = dx_g * cos_pt + dy_g * sin_pt
            if lx < -reach_limit:
                missed_count += 1
            else:
                active_queue.append((wx, wy, w_idx))
        weed_queue = active_queue

        # 3. Sort queue if idle
        if len(weed_queue) > 1 and arm_state == "HOME":
            sorted_queue = []
            curr_loc = np.array([0.0, 0.0])
            unvisited = list(weed_queue)
            while unvisited:
                local_pts = []
                for wx, wy, w_idx in unvisited:
                    dx_g = wx - px
                    dy_g = wy - py
                    lx = dx_g * cos_pt + dy_g * sin_pt
                    ly = -dx_g * sin_pt + dy_g * cos_pt
                    local_pts.append((lx, ly, wx, wy, w_idx))
                
                closest = min(local_pts, key=lambda p: np.hypot(p[0] - curr_loc[0], p[1] - curr_loc[1]))
                sorted_queue.append((closest[2], closest[3], closest[4]))
                for item in unvisited:
                    if item[2] == closest[4]:
                        unvisited.remove(item)
                        break
                curr_loc = np.array([closest[0], closest[1]])
            weed_queue = sorted_queue

        # 5. Speed planning (deadlock-free)
        # Check how many weeds in the queue are ALREADY in the arm's reach
        reachable_q_len = 0
        for wx, wy, w_idx in weed_queue:
            dx_g = wx - px
            dy_g = wy - py
            lx = dx_g * cos_pt + dy_g * sin_pt
            ly = -dx_g * sin_pt + dy_g * cos_pt
            if math.hypot(lx, ly) <= reach_limit:
                reachable_q_len += 1
                
        needs_stopping = (arm_state != "HOME") or (reachable_q_len > 0)
        
        # 4. Base navigation planning
        target_wp = waypoints[current_wp_idx]
        nav_dx = target_wp[0] - px
        nav_dy = target_wp[1] - py
        nav_dist = math.hypot(nav_dx, nav_dy)
        
        if nav_dist < REACH_THRESHOLD:
            current_wp_idx += 1
            continue
            
        target_heading = math.atan2(nav_dy, nav_dx)
        heading_err = target_heading - pt
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
        
        base_omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_HEADING * heading_err))
        
        # Velocity coordination
        base_fwd_speed = 0.0
        if coordination_mode == "STOP_AND_CUT":
            if needs_stopping:
                base_fwd_speed = 0.0
                base_omega = 0.0
            else:
                base_fwd_speed = cruise_speed if abs(heading_err) < 0.1 else MAX_SPEED
        elif coordination_mode == "CONTINUOUS_TRACKING":
            base_fwd_speed = cruise_speed if abs(heading_err) < 0.1 else MAX_SPEED
        elif coordination_mode == "DENSITY_ADAPTIVE_SPEED":
            # Scale speed factor based on weeds in reach
            q_active = reachable_q_len
            if arm_state != "HOME":
                q_active += 1
            speed_factor = max(0.0, 1.0 - q_active / 2.0)
            base_fwd_speed = cruise_speed * speed_factor if abs(heading_err) < 0.1 else MAX_SPEED * speed_factor
            
        # Update base position
        base_fwd_speed *= max(0.0, math.cos(heading_err))
        vx_g = base_fwd_speed * math.cos(pt)
        vy_g = base_fwd_speed * math.sin(pt)
        px += vx_g * dt
        py += vy_g * dt
        pt += base_omega * dt
        
        # Calculate wheel mechanical work
        wheel_energy += (abs(base_fwd_speed) * 120.0 + abs(base_omega) * 40.0) * dt # Proxy power (Watts) * dt

        # 6. Arm controller state machine
        if arm_state == "HOME":
            if len(weed_queue) > 0:
                # Check if first weed is in reach
                wx, wy, w_idx = weed_queue[0]
                dx_g = wx - px
                dy_g = wy - py
                lx = dx_g * cos_pt + dy_g * sin_pt
                ly = -dx_g * sin_pt + dy_g * cos_pt
                if math.hypot(lx, ly) <= reach_limit:
                    active_target_global = weed_queue.pop(0)
                    arm_state = "APPROACH"
                    arm_timer = 0.0
            else:
                # Command home
                if arm_type == "delta":
                    delta_x, delta_y, delta_z = 0.0, 0.0, Z_CLEARANCE / 1000.0
                else:
                    s1, s2, s3 = 0.0, 0.0, 0.0
                    
        if arm_state != "HOME":
            wx, wy, w_idx = active_target_global
            dx_g = wx - px
            dy_g = wy - py
            lx = dx_g * cos_pt + dy_g * sin_pt
            ly = -dx_g * sin_pt + dy_g * cos_pt
            
            # Check if target slipped past reach limit during cutting
            if math.hypot(lx, ly) > reach_limit + 0.04: # Allow buffer
                missed_count += 1
                arm_state = "HOME"
                active_target_global = None
                continue
                
            target_z = Z_CLEARANCE if arm_state != "PLUNGE" else Z_CUT
            
            if arm_type == "delta":
                # Delta Cartesian linear tracking
                tx, ty, tz = lx, ly, target_z / 1000.0
                
                # Apply velocity limits
                err_x, err_y, err_z = tx - delta_x, ty - delta_y, tz - delta_z
                dist_err = math.sqrt(err_x**2 + err_y**2 + err_z**2)
                step_dist = (ARM_MAX_VEL_MM_S / 1000.0) * dt
                
                if dist_err <= step_dist:
                    delta_x, delta_y, delta_z = tx, ty, tz
                else:
                    delta_x += (err_x / dist_err) * step_dist
                    delta_y += (err_y / dist_err) * step_dist
                    delta_z += (err_z / dist_err) * step_dist
                
                arm_energy += 38.0 * dt
                
                if dist_err < 0.01: # 10mm tolerance
                    if arm_state == "APPROACH":
                        arm_state = "PLUNGE"
                    elif arm_state == "PLUNGE":
                        arm_state = "RETRACT"
                        cut_count += 1
                    elif arm_state == "RETRACT":
                        arm_state = "HOME"
                        active_target_global = None
                        
            else:
                # Serial Joint angular tracking
                t1, t2, t3 = serial_kin.inverse(lx*1000, ly*1000, target_z)
                if t1 is None:
                    missed_count += 1
                    arm_state = "HOME"
                    active_target_global = None
                    continue
                
                # Apply motor joint speed limits
                err_s1 = t1 - s1
                s1 += np.clip(err_s1, -JOINT_MAX_VEL_RAD_S * dt, JOINT_MAX_VEL_RAD_S * dt)
                err_s2 = t2 - s2
                s2 += np.clip(err_s2, -JOINT_MAX_VEL_RAD_S * dt, JOINT_MAX_VEL_RAD_S * dt)
                err_s3 = t3 - s3
                s3 += np.clip(err_s3, -JOINT_MAX_VEL_RAD_S * dt, JOINT_MAX_VEL_RAD_S * dt)
                
                arm_energy += 42.0 * dt
                
                total_angle_err = abs(t1-s1) + abs(t2-s2) + abs(t3-s3)
                if total_angle_err < 0.08: # 0.08 rad (~4.5 deg) tolerance
                    if arm_state == "APPROACH":
                        arm_state = "PLUNGE"
                    elif arm_state == "PLUNGE":
                        arm_state = "RETRACT"
                        cut_count += 1
                    elif arm_state == "RETRACT":
                        arm_state = "HOME"
                        active_target_global = None
                        
            arm_timer += dt
            
        t += dt
        
    return {
        "efficiency": 100.0 * cut_count / (cut_count + missed_count + 1e-9),
        "cut": cut_count,
        "missed": missed_count,
        "time_s": t,
        "energy_joules": wheel_energy + arm_energy,
        "avg_speed": px / t if t > 0 else 0.0
    }

# --- RUN BATCH ---
def main():
    experiments = [
        # Arm, Mode, Cruise Speed
        ("delta", "STOP_AND_CUT", 0.4),
        ("delta", "CONTINUOUS_TRACKING", 0.1),
        ("delta", "CONTINUOUS_TRACKING", 0.2),
        ("delta", "CONTINUOUS_TRACKING", 0.3),
        ("delta", "CONTINUOUS_TRACKING", 0.4),
        ("delta", "DENSITY_ADAPTIVE_SPEED", 0.4), # DASC (scales down from 0.4)
        
        ("serial", "STOP_AND_CUT", 0.4),
        ("serial", "CONTINUOUS_TRACKING", 0.1),
        ("serial", "CONTINUOUS_TRACKING", 0.2),
        ("serial", "CONTINUOUS_TRACKING", 0.3),
        ("serial", "CONTINUOUS_TRACKING", 0.4),
        ("serial", "DENSITY_ADAPTIVE_SPEED", 0.4)
    ]
    
    results = []
    
    print("Running comparative coordination experiments...")
    for arm, mode, speed in experiments:
        res = run_simulation(arm, mode, speed)
        results.append({
            "arm_type": arm,
            "mode": mode,
            "cruise_speed": speed,
            "efficiency_pct": round(res["efficiency"], 1),
            "cut_count": res["cut"],
            "missed_count": res["missed"],
            "time_s": round(res["time_s"], 1),
            "energy_kj": round(res["energy_joules"] / 1000.0, 2)
        })
        label = f"{mode} ({speed} m/s)" if mode == "CONTINUOUS_TRACKING" else f"{mode}"
        print(f"  {arm.upper()} + {label}: Efficiency={res['efficiency']:.1f}%, Cut={res['cut']}, Missed={res['missed']}, Time={res['time_s']:.1f}s, Energy={res['energy_joules']/1000.0:.2f}kJ")
        
    # Save to CSV
    os.makedirs("results", exist_ok=True)
    out_path = "results/coordination_comparison.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    print(f"Saved results table to {out_path}")
    
    # Plot grouped bar chart comparison
    labels = ["Stop-and-Cut", "Cont. 0.1m/s", "Cont. 0.2m/s", "Cont. 0.3m/s", "Cont. 0.4m/s", "DASC (Ours)"]
    
    delta_eff = [r["efficiency_pct"] for r in results if r["arm_type"] == "delta"]
    serial_eff = [r["efficiency_pct"] for r in results if r["arm_type"] == "serial"]
    
    delta_time = [r["time_s"] for r in results if r["arm_type"] == "delta"]
    serial_time = [r["time_s"] for r in results if r["arm_type"] == "serial"]

    delta_energy = [r["energy_kj"] for r in results if r["arm_type"] == "delta"]
    serial_energy = [r["energy_kj"] for r in results if r["arm_type"] == "serial"]

    x = np.arange(len(labels))
    width = 0.35
    
    # Figure 1: Weeding Efficiency
    plt.figure(figsize=(10, 5))
    plt.bar(x - width/2, delta_eff, width, label='Delta Robot', color='steelblue')
    plt.bar(x + width/2, serial_eff, width, label='Serial Arm', color='lightsalmon')
    plt.ylabel('Weeding Excision Success Rate (%)')
    plt.title('Weeding Efficiency Comparison')
    plt.xticks(x, labels)
    plt.ylim(0, 110)
    plt.grid(axis='y', ls=':', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/comparison_efficiency.png", dpi=300)
    plt.close()

    # Figure 2: Operational Time and Energy
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    ax2.bar(x - width/2, delta_time, width, label='Delta Robot', color='steelblue')
    ax2.bar(x + width/2, serial_time, width, label='Serial Arm', color='lightsalmon')
    ax2.set_ylabel('Total Mission Duration (s)')
    ax2.set_title('Mission Completion Time')
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels)
    ax2.grid(axis='y', ls=':', alpha=0.6)
    ax2.legend()
    
    ax1.bar(x - width/2, delta_energy, width, label='Delta Robot', color='steelblue')
    ax1.bar(x + width/2, serial_energy, width, label='Serial Arm', color='lightsalmon')
    ax1.set_ylabel('Total Mechanical Energy Consumption (kJ)')
    ax1.set_title('Total Energy Footprint')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.grid(axis='y', ls=':', alpha=0.6)
    ax1.legend()
    
    plt.tight_layout()
    plt.savefig("results/comparison_time_energy.png", dpi=300)
    plt.close()
    print("Saved comparison plots to results/")

if __name__ == "__main__":
    main()
