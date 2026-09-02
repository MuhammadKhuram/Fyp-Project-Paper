
import numpy as np


import config as cfg

# Core IK/FK for the delta robot, plus the figure/table generation for the
# paper. This is the one place the kinematics math lives - integration.py
# imports DeltaKinematics from here instead of rewriting it (we already
# made that mistake once, not doing it again).
#
# Everything gets saved to config.RESULTS_DIR automatically - run this
# file and check the results/ folder.


class DeltaKinematics:
    def __init__(self):
        self.sb, self.sp = cfg.BASE_SIDE, cfg.PLATFORM_SIDE
        self.L, self.l = cfg.UPPER_ARM, cfg.LOWER_ARM
        self.wb = (np.sqrt(3) / 6) * self.sb
        self.wp = (np.sqrt(3) / 6) * self.sp
        self.a = self.wb - self.wp

        self.theta_min = cfg.THETA_MIN
        self.theta_max = cfg.THETA_MAX
        self.min_wrist_clearance = cfg.MIN_WRIST_CLEARANCE

    def _solve_leg_raw(self, x, y, z):
        E, F = 2 * self.L * (y + self.a), 2 * z * self.L
        G = x**2 + y**2 + z**2 + self.a**2 + self.L**2 + 2 * y * self.a - self.l**2
        dist = E**2 + F**2
        if G**2 > dist:
            return None
        t = (-F - np.sqrt(dist - G**2)) / (G - E)
        return np.degrees(2 * np.arctan(t))

    def inverse(self, x0, y0, z0, enforce_limits=True, check_collision=True):
        """Target (x, y, z) in the delta's own local frame -> 3 joint
        angles in degrees. Any entry comes back None if that leg can't
        reach it, blows past the joint limit, or trips the collision
        check."""
        c120, s120 = -0.5, np.sqrt(3) / 2
        t1 = self._solve_leg_raw(x0, y0, z0)
        t2 = self._solve_leg_raw(x0 * c120 + y0 * s120, x0 * -s120 + y0 * c120, z0)
        t3 = self._solve_leg_raw(x0 * c120 - y0 * s120, x0 * s120 + y0 * c120, z0)
        angles = [t1, t2, t3]

        if enforce_limits:
            angles = [a if (a is not None and self.theta_min <= a <= self.theta_max) else None
                      for a in angles]

        if check_collision and all(a is not None for a in angles):
            if not self.check_self_clearance(*angles):
                return None, None, None

        return tuple(angles)

    def forward(self, t1, t2, t3):
        if any(t is None for t in [t1, t2, t3]):
            return None

        def _wrist(t_deg, phi_deg):
            phi, t = np.radians(phi_deg), np.radians(t_deg)
            y_loc, z = -(self.a + self.L * np.cos(t)), -self.L * np.sin(t)
            return np.array([-y_loc * np.sin(phi), y_loc * np.cos(phi), z])

        p1, p2, p3 = _wrist(t1, 0), _wrist(t2, 120), _wrist(t3, 240)
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        x3, y3, z3 = p3

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        if abs(dnm) < 1e-9:
            return None

        w1, w2, w3 = np.sum(p1**2), np.sum(p2**2), np.sum(p3**2)
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        a2 = -((z2 - z1) * x3 - (z3 - z1) * x2)
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        A = (a1 / dnm)**2 + (a2 / dnm)**2 + 1
        B = 2 * ((a1 / dnm) * (b1 / dnm - x1) + (a2 / dnm) * (b2 / dnm - y1) - z1)
        C = (b1 / dnm - x1)**2 + (b2 / dnm - y1)**2 + z1**2 - self.l**2

        delta = B**2 - 4 * A * C
        if delta < 0:
            return None
        z = (-B - np.sqrt(delta)) / (2 * A)
        return (a1 * z + b1) / dnm, (a2 * z + b2) / dnm, z

    def _wrist_point(self, t_deg, phi_deg):
        phi, t = np.radians(phi_deg), np.radians(t_deg)
        y_loc, z = -(self.a + self.L * np.cos(t)), -self.L * np.sin(t)
        return np.array([-y_loc * np.sin(phi), y_loc * np.cos(phi), z])

    def check_self_clearance(self, t1, t2, t3):
        p1 = self._wrist_point(t1, 0)
        p2 = self._wrist_point(t2, 120)
        p3 = self._wrist_point(t3, 240)
        d12 = np.linalg.norm(p1 - p2)
        d23 = np.linalg.norm(p2 - p3)
        d13 = np.linalg.norm(p1 - p3)
        return min(d12, d23, d13) >= self.min_wrist_clearance
