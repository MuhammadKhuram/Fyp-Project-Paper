import math
import numpy as np
from controller import Supervisor

import config as cfg
from delta_robot_kinematics import DeltaKinematics
from test_data_generator import generate_robot_frame_layout

TIME_STEP = 32
WORLD_OFFSET_MM = np.array([0.0, 0.0, 300.0])

# Speeds and Heights (Adapted for physical limits)
V_APPROACH_MM_S = 80.0
V_PLUNGE_MM_S = 40.0
V_RETRACT_MM_S = 80.0
Z_HOVER = -145.0  
Z_CUT = -200.0    
DWELL_TIME_S = 0.3  

def outward(phi_deg):
    p = math.radians(phi_deg)
    return np.array([math.sin(p), -math.cos(p), 0.0])

def orient_solid(node, point_a_mm, point_b_mm, fixed_axis=None):
    a_m, b_m = np.array(point_a_mm) / 1000.0, np.array(point_b_mm) / 1000.0
    direction = b_m - a_m
    length = np.linalg.norm(direction)
    if length < 1e-9: return
    direction = direction / length

    Z = np.array([0.0, 0.0, 1.0])
    
    if fixed_axis is not None:
        # Prevent 3D Gimbal Lock twist by locking the rotation to the physical hinge pin
        sin_angle = np.dot(fixed_axis, np.cross(Z, direction))
        cos_angle = np.dot(Z, direction)
        angle = math.atan2(sin_angle, cos_angle)
        rotation = [fixed_axis[0], fixed_axis[1], fixed_axis[2], angle]
    else:
        # Standard free rotation for ball joints (forearms)
        dot = np.clip(np.dot(Z, direction), -1.0, 1.0)
        axis = np.cross(Z, direction)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            rotation = [1.0, 0.0, 0.0, 0.0] if dot > 0 else [1.0, 0.0, 0.0, math.pi]
        else:
            axis = axis / axis_norm
            rotation = [axis[0], axis[1], axis[2], math.acos(dot)]

    node.getField("translation").setSFVec3f(((a_m + b_m) / 2.0).tolist())
    node.getField("rotation").setSFRotation(rotation)

class LegRig:
    def __init__(self, supervisor, leg_num, phi_deg, kinematics):
        self.kin = kinematics
        self.phi = phi_deg
        self.upper_arm = supervisor.getFromDef(f"UPPER_ARM_{leg_num}")
        self.forearm_a = supervisor.getFromDef(f"FOREARM_{leg_num}A")
        self.forearm_b = supervisor.getFromDef(f"FOREARM_{leg_num}B")
        self.base_joint = self.kin.wb * outward(self.phi)

    def draw(self, theta, platform_target_mm):
        if theta is None: return
        u = outward(self.phi)
        raw_wrist = self.kin._wrist_point(theta, self.phi)
        true_wrist = raw_wrist + self.kin.wp * u
        platform_pt = platform_target_mm + self.kin.wp * u
        lateral_vec = np.cross(np.array([0.0, 0.0, 1.0]), u) * 15.0

        # Calculate the exact physical hinge pin axis to prevent twisting
        tangent_axis = np.cross(np.array([0.0, 0.0, 1.0]), u)
        tangent_axis = tangent_axis / np.linalg.norm(tangent_axis)

        # Upper arm is locked to the hinge. Forearms use free-spinning ball joints.
        orient_solid(self.upper_arm, self.base_joint + WORLD_OFFSET_MM, true_wrist + WORLD_OFFSET_MM, fixed_axis=tangent_axis)
        orient_solid(self.forearm_a, true_wrist + WORLD_OFFSET_MM + lateral_vec, platform_pt + WORLD_OFFSET_MM + lateral_vec)
        orient_solid(self.forearm_b, true_wrist + WORLD_OFFSET_MM - lateral_vec, platform_pt + WORLD_OFFSET_MM - lateral_vec)

def main():
    supervisor = Supervisor()
    kin = DeltaKinematics()
    legs = [LegRig(supervisor, 1, 0.0, kin), LegRig(supervisor, 2, 120.0, kin), LegRig(supervisor, 3, 240.0, kin)]
    platform = supervisor.getFromDef("PLATFORM")
    target_marker = supervisor.getFromDef("TARGET")

    # 1. Generate Random Weeds
    print("Generating 6 random weed coordinates...")
    unsorted_weeds = generate_robot_frame_layout(n_weeds=6, seed=42, z_fixed=Z_CUT)
    
    # 2. Greedy Nearest-Neighbor Sort
    curr_xy = np.array([0.0, 0.0])
    sorted_weeds = []
    unvisited = list(unsorted_weeds)
    while unvisited:
        nxt = min(unvisited, key=lambda w: math.hypot(w[0]-curr_xy[0], w[1]-curr_xy[1]))
        sorted_weeds.append(nxt)
        unvisited.remove(nxt)
        curr_xy = np.array([nxt[0], nxt[1]])
        
    print("Optimized Path Planned:")
    for i, w in enumerate(sorted_weeds):
        print(f" {i+1}. X:{w[0]:.1f}, Y:{w[1]:.1f}")

    # State Machine Variables
    home_pt = np.array([0.0, 0.0, Z_HOVER])
    current_pt = np.copy(home_pt)
    target_pt = np.copy(home_pt)
    
    weed_queue = list(sorted_weeds)
    active_weed = None
    
    state = "IDLE"
    state_start_time = supervisor.getTime()
    move_duration = 0.0

    while supervisor.step(TIME_STEP) != -1:
        t = supervisor.getTime()
        
        # Default to staying still to prevent crashes while waiting
        active_pt = np.copy(current_pt)
        
        # ==========================================
        # STATE MACHINE (Milestone C: Excision Sequence)
        # ==========================================
        if state == "IDLE":
            if t > 1.0: # Wait 1 sec before starting
                if weed_queue:
                    active_weed = weed_queue.pop(0)
                    target_pt = np.array([active_weed[0], active_weed[1], Z_HOVER])
                    
                    dist = np.linalg.norm(target_pt - current_pt)
                    move_duration = dist / V_APPROACH_MM_S
                    
                    state = "APPROACHING"
                    state_start_time = t
                    print(f"\nMoving to Weed at X:{active_weed[0]:.0f}, Y:{active_weed[1]:.0f}")
                else:
                    target_pt = np.copy(home_pt)
                    dist = np.linalg.norm(target_pt - current_pt)
                    move_duration = dist / V_APPROACH_MM_S
                    state = "DONE"
                    state_start_time = t
                    print("\nAll weeds cleared. Returning home.")
                    
        elif state in ["APPROACHING", "PLUNGING", "RETRACTING", "DONE"]:
            progress = (t - state_start_time) / move_duration if move_duration > 0 else 1.0
            
            if progress >= 1.0:
                progress = 1.0
                current_pt = np.copy(target_pt)
                
                # State Transitions
                if state == "APPROACHING":
                    target_pt = np.array([active_weed[0], active_weed[1], Z_CUT])
                    move_duration = abs(target_pt[2] - current_pt[2]) / V_PLUNGE_MM_S
                    state = "PLUNGING"
                    state_start_time = t
                    
                elif state == "PLUNGING":
                    state = "DWELLING"
                    state_start_time = t
                    
                elif state == "RETRACTING":
                    state = "IDLE" # Ready for next weed
                    
            # Linear Interpolation
            active_pt = current_pt + (target_pt - current_pt) * progress
            
        elif state == "DWELLING":
            active_pt = target_pt
            if t - state_start_time > DWELL_TIME_S:
                current_pt = np.copy(target_pt)
                target_pt = np.array([active_weed[0], active_weed[1], Z_HOVER])
                move_duration = abs(target_pt[2] - current_pt[2]) / V_RETRACT_MM_S
                state = "RETRACTING"
                state_start_time = t

        # ==========================================
        # KINEMATICS & DRAWING
        # ==========================================
        x, y, z = active_pt
        t1, t2, t3 = kin.inverse(x, y, z)
        
        if t1 is not None and t2 is not None and t3 is not None:
            legs[0].draw(t1, active_pt)
            legs[1].draw(t2, active_pt)
            legs[2].draw(t3, active_pt)
            
            world_pos = (active_pt + WORLD_OFFSET_MM) / 1000.0
            platform.getField("translation").setSFVec3f(world_pos.tolist())
            target_marker.getField("translation").setSFVec3f(world_pos.tolist())

if __name__ == "__main__":
    main()
