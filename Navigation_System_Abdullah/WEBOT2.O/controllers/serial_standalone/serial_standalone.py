import sys
import os
import math
from controller import Robot

# Add parent workspace to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

TIME_STEP = 32
robot = Robot()

print("[SerialStandalone] Starting standalone controller...")

# Rotational motors
arm_motor_1 = robot.getDevice("arm_motor_1") # base yaw
arm_motor_2 = robot.getDevice("arm_motor_2") # shoulder pitch
arm_motor_3 = robot.getDevice("arm_motor_3") # elbow pitch

# Set limits
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
        """
        t1 = math.atan2(y, x)
        r = math.hypot(x, y)
        z_rel = z - self.z_offset
        cos_t3 = (r**2 + z_rel**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if cos_t3 < -1.0 or cos_t3 > 1.0:
            return None, None, None
        t3 = math.acos(cos_t3)
        t2 = math.atan2(r, -z_rel) - math.atan2(self.L2 * math.sin(t3), self.L1 + self.L2 * math.cos(t3))
        return t1, t2, t3

arm_kin = SerialArmKinematics()

t = 0.0
dt = TIME_STEP / 1000.0

# Initial home pose
arm_motor_1.setPosition(0.0)
arm_motor_2.setPosition(0.0)
arm_motor_3.setPosition(0.0)

while robot.step(TIME_STEP) != -1:
    t += dt
    
    # 1. Define pre-programmed test trajectory (circle sweep + plunge)
    # Radius = 80mm, Z sweeps from -50mm (clearance) down to -170mm (excision)
    radius_mm = 80.0
    omega = 1.0 # rad/s
    
    target_x_mm = radius_mm * math.cos(omega * t)
    target_y_mm = radius_mm * math.sin(omega * t)
    
    # Plunging cycle every 6 seconds
    cycle_time = t % 6.0
    if cycle_time < 2.0:
        # Phase 1: Hold clearance
        target_z_mm = -50.0
    elif cycle_time < 3.0:
        # Phase 2: Plunge down
        target_z_mm = -50.0 - (120.0 * (cycle_time - 2.0))
    elif cycle_time < 4.0:
        # Phase 3: Dwell
        target_z_mm = -170.0
    else:
        # Phase 4: Retract back
        target_z_mm = -170.0 + (120.0 * (cycle_time - 4.0))

    # 2. Command rotational motors using Analytical IK
    t1, t2, t3 = arm_kin.inverse(target_x_mm, target_y_mm, target_z_mm)
    
    if t1 is not None:
        arm_motor_1.setPosition(t1)
        arm_motor_2.setPosition(t2)
        arm_motor_3.setPosition(t3)
    
    # Log to console throttled
    if int(t * 10) % 20 == 0:
        print(f"t={t:.1f}s Target Cartesian=[{target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f}] mm")
        if t1 is not None:
            print(f"  Calculated Joint Angles: t1={math.degrees(t1):.1f}°, t2={math.degrees(t2):.1f}°, t3={math.degrees(t3):.1f}°")
        else:
            print("  Target is UNREACHABLE!")
