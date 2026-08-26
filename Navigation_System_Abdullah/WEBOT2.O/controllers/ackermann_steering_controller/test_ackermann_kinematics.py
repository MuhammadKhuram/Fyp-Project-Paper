import math
import unittest

def compute_ackermann_angles(steering_angle_center, wheelbase=0.6, track_width=0.64, max_steer=math.radians(45)):
    """
    Computes inner and outer front wheel steering angles based on Ackermann kinematics.
    steering_angle_center: virtual central steering angle in radians (positive = turning left)
    wheelbase (L): distance between front and rear axles (meters)
    track_width (W): distance between left and right wheels (meters)
    max_steer: maximum allowable central steering angle magnitude in radians (default 45 degrees)
    """
    # Clamp virtual central steering angle to max limits [-45°, +45°]
    delta = max(-max_steer, min(max_steer, steering_angle_center))
    
    if abs(delta) < 1e-6:
        return 0.0, 0.0, delta

    # Turning radius from center of rear axle to center of turn
    R = wheelbase / math.tan(abs(delta))

    # Inner and outer wheel angles
    delta_inner = math.atan(wheelbase / (R - track_width / 2.0))
    delta_outer = math.atan(wheelbase / (R + track_width / 2.0))

    if delta > 0:
        # Turning Left: Left wheel is inner, Right wheel is outer
        return delta_inner, delta_outer, delta
    else:
        # Turning Right: Right wheel is inner (-), Left wheel is outer (-)
        return -delta_outer, -delta_inner, delta

class TestAckermannKinematics(unittest.TestCase):
    def test_straight_line(self):
        left_angle, right_angle, center_clamped = compute_ackermann_angles(0.0)
        self.assertAlmostEqual(left_angle, 0.0)
        self.assertAlmostEqual(right_angle, 0.0)

    def test_left_turn_45_deg(self):
        # 45 degrees = ~0.785398 rad
        center_input = math.radians(45)
        left_angle, right_angle, center_clamped = compute_ackermann_angles(center_input)
        
        # Turning left -> left wheel is inner (larger angle), right wheel is outer (smaller angle)
        self.assertGreater(left_angle, right_angle)
        self.assertGreater(left_angle, 0)
        self.assertGreater(right_angle, 0)
        self.assertAlmostEqual(center_clamped, center_input)

    def test_clamping_exceed_limits(self):
        # Input 60 degrees (exceeds 45 degrees limit)
        center_input = math.radians(60)
        max_45 = math.radians(45)
        left_angle, right_angle, center_clamped = compute_ackermann_angles(center_input)
        
        self.assertAlmostEqual(center_clamped, max_45)

    def test_right_turn_negative(self):
        center_input = math.radians(-30)
        left_angle, right_angle, center_clamped = compute_ackermann_angles(center_input)
        
        # Turning right -> both angles are negative, right wheel is inner (more negative)
        self.assertLess(left_angle, 0)
        self.assertLess(right_angle, 0)
        self.assertLess(right_angle, left_angle)

if __name__ == '__main__':
    unittest.main()
