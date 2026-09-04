import math
import numpy as np
import cv2  
from ultralytics import YOLO  
from controller import Supervisor

import config as cfg
from delta_robot_kinematics import DeltaKinematics

TIME_STEP = 32
WORLD_OFFSET_MM = np.array([0.0, 0.0, 300.0])

V_APPROACH_MM_S = 250.0  
V_PLUNGE_MM_S = 150.0
V_RETRACT_MM_S = 250.0
Z_HOVER = -145.0  
Z_CUT = -200.0    
DWELL_TIME_S = 0.15  
ROVER_SPEED_MM_S = 80.0  

CAM_WIDTH, CAM_HEIGHT = 640, 480
CAM_FOV_RAD = 1.2
FOCAL_LENGTH_PX = (CAM_WIDTH / 2) / math.tan(CAM_FOV_RAD / 2)
CAM_MOUNT_X_MM = 400.0  

def outward(phi_deg):
    p = math.radians(phi_deg)
    return np.array([math.sin(p), -math.cos(p), 0.0])

def orient_solid(node, point_a_mm, point_b_mm):
    a_m, b_m = np.array(point_a_mm) / 1000.0, np.array(point_b_mm) / 1000.0
    direction = b_m - a_m
    length = np.linalg.norm(direction)
    if length < 1e-9: return
    direction = direction / length

    default_axis = np.array([0.0, 0.0, 1.0])
    dot = np.clip(np.dot(default_axis, direction), -1.0, 1.0)
    axis = np.cross(default_axis, direction)
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
        lateral_vec = np.cross(np.array([0.0, 0.0, 1.0]), u)
        offset = lateral_vec * 15.0

        orient_solid(self.upper_arm, self.base_joint + WORLD_OFFSET_MM, true_wrist + WORLD_OFFSET_MM)
        orient_solid(self.forearm_a, true_wrist + WORLD_OFFSET_MM + offset, platform_pt + WORLD_OFFSET_MM + offset)
        orient_solid(self.forearm_b, true_wrist + WORLD_OFFSET_MM - offset, platform_pt + WORLD_OFFSET_MM - offset)

def get_reachable_ik(kin, x, y, z):
    r = math.hypot(x, y)
    orig_r = r if r > 0 else 1.0
    t1, t2, t3 = kin.inverse(x, y, z, enforce_limits=False, check_collision=False)
    
    while (t1 is None or t2 is None or t3 is None) and r > 10.0:
        r -= 5.0  
        x = (x / orig_r) * r
        y = (y / orig_r) * r
        t1, t2, t3 = kin.inverse(x, y, z, enforce_limits=False, check_collision=False)
        
    return x, y, z, t1, t2, t3

def main():
    supervisor = Supervisor()
    rgb_camera = supervisor.getDevice("rgb_camera")
    rgb_camera.enable(TIME_STEP)
    depth_camera = supervisor.getDevice("depth_camera")
    depth_camera.enable(TIME_STEP)
    
    crop_field = supervisor.getFromDef("CROP_FIELD")
    field_trans_node = crop_field.getField("translation")
    
    kin = DeltaKinematics()
    legs = [LegRig(supervisor, 1, 0.0, kin), LegRig(supervisor, 2, 120.0, kin), LegRig(supervisor, 3, 240.0, kin)]
    platform = supervisor.getFromDef("PLATFORM")
    target_marker = supervisor.getFromDef("TARGET")

    print("Loading YOLOv8 model...")
    model = YOLO("/home/muhammad-khuram-linux/best.pt")
    
    home_pt = np.array([0.0, 0.0, Z_HOVER])
    current_pt = np.copy(home_pt)
    target_pt = np.copy(home_pt)
    
    weed_queue = []
    active_weed = None
    state = "IDLE"
    state_start_time = supervisor.getTime()
    move_duration = 0.0
    frame_counter = 0

    weeds_cut = 0
    weeds_missed = 0
    total_errors_mm = []

    print("\n==============================================")
    print("AWRR SIMULATION ONLINE")
    print("Tracking: End-Effector Precision & Efficiency")
    print("==============================================\n")

    while supervisor.step(TIME_STEP) != -1:
        t = supervisor.getTime()
        dt_s = TIME_STEP / 1000.0
        dx_mm = ROVER_SPEED_MM_S * dt_s  
        frame_counter += 1
        
        pos = field_trans_node.getSFVec3f()
        pos[0] -= dx_mm / 1000.0 
        field_trans_node.setSFVec3f(pos)
        
        for w in weed_queue:
            w[0] -= dx_mm
            
        # Do not shift coordinates if we are idle or returning to the center!
        if state not in ["IDLE", "DONE", "RETURNING_HOME"]:
            current_pt[0] -= dx_mm
            target_pt[0] -= dx_mm
            if active_weed is not None:
                active_weed[0] -= dx_mm

        active_pt = np.copy(current_pt)
        
        if frame_counter % 4 == 0:
            raw_image = rgb_camera.getImage()
            if raw_image:
                img_array = np.frombuffer(raw_image, np.uint8).reshape((CAM_HEIGHT, CAM_WIDTH, 4))
                bgr_frame = img_array[:, :, :3]
                results = model(bgr_frame, verbose=False)
                depth_raw = depth_camera.getRangeImage()
                depth_array = np.array(depth_raw, dtype=np.float32).reshape((CAM_HEIGHT, CAM_WIDTH))
                
                for box in results[0].boxes:
                    label = results[0].names[int(box.cls[0].item())].lower()
                    if "weed" in label and box.conf[0].item() > 0.4:
                        coords = box.xyxy[0].cpu().numpy()
                        u = int((coords[0] + coords[2]) / 2)
                        v = int((coords[1] + coords[3]) / 2)
                        depth_m = depth_array[v, u]
                        
                        if 0.1 < depth_m < 1.5:
                            offset_x_m = (240 - v) * depth_m / FOCAL_LENGTH_PX
                            offset_y_m = (320 - u) * depth_m / FOCAL_LENGTH_PX
                            weed_x = CAM_MOUNT_X_MM + (offset_x_m * 1000.0)
                            weed_y = offset_y_m * 1000.0
                            
                            if abs(weed_y) > 150.0:
                                continue
                            
                            is_dup = False
                            for w in weed_queue:
                                if math.hypot(weed_x - w[0], weed_y - w[1]) < 60.0:
                                    is_dup = True; break
                            if active_weed is not None and math.hypot(weed_x - active_weed[0], weed_y - active_weed[1]) < 60.0:
                                is_dup = True
                                
                            if not is_dup:
                                weed_queue.append(np.array([weed_x, weed_y, Z_CUT]))

                cv2.imshow("AWRR Live Vision", results[0].plot())
                cv2.waitKey(1)
        
        if active_weed is not None and active_weed[0] < -120.0 and state in ["APPROACHING", "PLUNGING", "DWELLING"]:
            target_pt = np.array([current_pt[0], current_pt[1], Z_HOVER])
            move_duration = abs(target_pt[2] - current_pt[2]) / V_RETRACT_MM_S
            state = "RETRACTING"
            state_start_time = t
            active_weed = None
            
            weeds_missed += 1
            print(f"❌ MISSED WEED! (Slid out of bounds)")
            eff = (weeds_cut / (weeds_cut + weeds_missed)) * 100.0
            print(f"📊 Stats | Cut: {weeds_cut} | Missed: {weeds_missed} | Efficiency: {eff:.1f}%\n")

        elif state == "IDLE":
            weed_targeted = False
            if weed_queue:
                for i, w in enumerate(weed_queue):
                    if -50.0 <= w[0] <= 150.0:  
                        if math.hypot(w[0], w[1]) <= 200.0:
                            active_weed = weed_queue.pop(i)
                            target_pt = np.array([active_weed[0], active_weed[1], Z_HOVER])
                            move_duration = np.linalg.norm(target_pt - current_pt) / V_APPROACH_MM_S
                            state = "APPROACHING"
                            state_start_time = t
                            weed_targeted = True
                            break
                    elif w[0] < -150.0:
                        weed_queue.pop(i)
                        weeds_missed += 1
                        print(f"❌ MISSED WEED! (Passed by unchecked)")
                        eff = (weeds_cut / (weeds_cut + weeds_missed)) * 100.0
                        print(f"📊 Stats | Cut: {weeds_cut} | Missed: {weeds_missed} | Efficiency: {eff:.1f}%\n")
                        break
            
            # GO HOME LOGIC: If no weed is ready, and we aren't at the center, go back to center!
            if not weed_targeted and math.hypot(current_pt[0], current_pt[1]) > 5.0:
                target_pt = np.copy(home_pt)
                move_duration = np.linalg.norm(target_pt - current_pt) / V_RETRACT_MM_S
                state = "RETURNING_HOME"
                state_start_time = t

        elif state in ["APPROACHING", "PLUNGING", "RETRACTING", "RETURNING_HOME"]:
            
            # SMART INTERRUPT: If we are going home but a weed enters the strike zone, attack it immediately!
            if state == "RETURNING_HOME" and weed_queue:
                for i, w in enumerate(weed_queue):
                    if -50.0 <= w[0] <= 150.0 and math.hypot(w[0], w[1]) <= 200.0:
                        current_pt = np.copy(active_pt)  # Stop mid-air
                        active_weed = weed_queue.pop(i)
                        target_pt = np.array([active_weed[0], active_weed[1], Z_HOVER])
                        move_duration = np.linalg.norm(target_pt - current_pt) / V_APPROACH_MM_S
                        state = "APPROACHING"
                        state_start_time = t
                        break

            progress = (t - state_start_time) / move_duration if move_duration > 0 else 1.0
            if progress >= 1.0:
                progress = 1.0
                current_pt = np.copy(target_pt)
                
                if state == "APPROACHING":
                    target_pt = np.array([active_weed[0], active_weed[1], Z_CUT])
                    move_duration = abs(target_pt[2] - current_pt[2]) / V_PLUNGE_MM_S
                    state = "PLUNGING"
                    state_start_time = t
                elif state == "PLUNGING":
                    state = "DWELLING"
                    state_start_time = t
                    
                    blade_pos = current_pt
                    weed_pos = active_weed
                    error_mm = math.hypot(blade_pos[0] - weed_pos[0], blade_pos[1] - weed_pos[1])
                    total_errors_mm.append(error_mm)
                    
                    weeds_cut += 1
                    avg_error = sum(total_errors_mm)/len(total_errors_mm)
                    eff = (weeds_cut / (weeds_cut + weeds_missed)) * 100.0
                    
                    print(f"✅ CUT CONFIRMED! Precision Error: {error_mm:.1f} mm")
                    print(f"📈 RMS Error: {avg_error:.1f} mm | Efficiency: {eff:.1f}%")
                    print(f"📊 Stats | Cut: {weeds_cut} | Missed: {weeds_missed}\n")
                    
                elif state == "RETRACTING" or state == "RETURNING_HOME":
                    state = "IDLE" 
                    active_weed = None
            active_pt = current_pt + (target_pt - current_pt) * progress
            
        elif state == "DWELLING":
            active_pt = target_pt
            if t - state_start_time > DWELL_TIME_S:
                current_pt = np.copy(target_pt)
                target_pt = np.array([active_weed[0], active_weed[1], Z_HOVER])
                move_duration = abs(target_pt[2] - current_pt[2]) / V_RETRACT_MM_S
                state = "RETRACTING"
                state_start_time = t

        safe_x, safe_y, safe_z, t1, t2, t3 = get_reachable_ik(kin, active_pt[0], active_pt[1], active_pt[2])
        
        if t1 is not None and t2 is not None and t3 is not None:
            legs[0].draw(t1, np.array([safe_x, safe_y, safe_z]))
            legs[1].draw(t2, np.array([safe_x, safe_y, safe_z]))
            legs[2].draw(t3, np.array([safe_x, safe_y, safe_z]))
            
            world_pos = (np.array([safe_x, safe_y, safe_z]) + WORLD_OFFSET_MM) / 1000.0
            platform.getField("translation").setSFVec3f(world_pos.tolist())
            target_marker.getField("translation").setSFVec3f(world_pos.tolist())

if __name__ == "__main__":
    main()
