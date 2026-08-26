import sys
import os
import math
from controller import Robot

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "Delta-Robot-Subsystem")))

from delta_robot_kinematics import DeltaKinematics

TIME_STEP = 32
robot = Robot()

print("=========================================================")
print("  STANDALONE 3-DOF PARALLEL DELTA ROBOT CONTROLLER")
print("=========================================================")

# Upper arm hinge motors (3 motors at 120 degree angles)
motor1 = robot.getDevice("motor_1")
motor2 = robot.getDevice("motor_2")
motor3 = robot.getDevice("motor_3")

# End-effector Cartesian slider motors
delta_x = robot.getDevice("delta_x_motor")
delta_y = robot.getDevice("delta_y_motor")
delta_z = robot.getDevice("delta_z_motor")

delta_x.setVelocity(0.4)
delta_y.setVelocity(0.4)
delta_z.setVelocity(0.4)

kin = DeltaKinematics()

t = 0.0
dt = TIME_STEP / 1000.0

while robot.step(TIME_STEP) != -1:
    t += dt
    
    # Circular workspace sweep (radius 80mm) + Plunge cycles
    radius_mm = 80.0
    omega = 1.2
    
    target_x_mm = radius_mm * math.cos(omega * t)
    target_y_mm = radius_mm * math.sin(omega * t)
    
    cycle_time = t % 6.0
    if cycle_time < 2.0:
        target_z_mm = -50.0 # clearance height (mm)
    elif cycle_time < 3.0:
        target_z_mm = -50.0 - (120.0 * (cycle_time - 2.0)) # plunging
    elif cycle_time < 4.0:
        target_z_mm = -170.0 # cutting depth
    else:
        target_z_mm = -170.0 + (120.0 * (cycle_time - 4.0)) # retracting

    # Command Cartesian sliders (in Webots meters)
    delta_x.setPosition(target_x_mm / 1000.0)
    delta_y.setPosition(target_y_mm / 1000.0)
    
    z_slider_m = (target_z_mm - (-50.0)) / 1000.0
    delta_z.setPosition(z_slider_m)

    # Calculate and set physical upper arm joint angles
    angles = kin.inverse(target_x_mm, target_y_mm, target_z_mm, enforce_limits=False)
    
    t1 = math.radians(angles[0]) if angles[0] is not None else 0.0
    t2 = math.radians(angles[1]) if angles[1] is not None else 0.0
    t3 = math.radians(angles[2]) if angles[2] is not None else 0.0
    
    motor1.setPosition(t1)
    motor2.setPosition(t2)
    motor3.setPosition(t3)
    
    if int(t * 10) % 20 == 0:
        print(f"t={t:.1f}s | Target Position [X:{target_x_mm:.1f}, Y:{target_y_mm:.1f}, Z:{target_z_mm:.1f}] mm")
        print(f"       | Bicep Actuator Angles: θ1={angles[0]:.1f}°, θ2={angles[1]:.1f}°, θ3={angles[2]:.1f}°")
