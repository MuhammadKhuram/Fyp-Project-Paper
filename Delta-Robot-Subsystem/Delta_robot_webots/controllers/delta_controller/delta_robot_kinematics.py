import numpy as np
import config as cfg

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
        c120, s120 = -0.5, np.sqrt(3) / 2
        t1 = self._solve_leg_raw(x0, y0, z0)
        t2 = self._solve_leg_raw(x0 * c120 + y0 * s120, x0 * -s120 + y0 * c120, z0)
        t3 = self._solve_leg_raw(x0 * c120 - y0 * s120, x0 * s120 + y0 * c120, z0)
        angles = [t1, t2, t3]
        if enforce_limits:
            angles = [a if (a is not None and self.theta_min <= a <= self.theta_max) else None for a in angles]
        if check_collision and all(a is not None for a in angles):
            if not self.check_self_clearance(*angles): return None, None, None
        return tuple(angles)

    def forward(self, t1, t2, t3): return None

    def _wrist_point(self, t_deg, phi_deg):
        phi, t = np.radians(phi_deg), np.radians(t_deg)
        y_loc, z = -(self.a + self.L * np.cos(t)), -self.L * np.sin(t)
        return np.array([-y_loc * np.sin(phi), y_loc * np.cos(phi), z])

    def check_self_clearance(self, t1, t2, t3):
        p1, p2, p3 = self._wrist_point(t1, 0), self._wrist_point(t2, 120), self._wrist_point(t3, 240)
        return min(np.linalg.norm(p1 - p2), np.linalg.norm(p2 - p3), np.linalg.norm(p1 - p3)) >= self.min_wrist_clearance
