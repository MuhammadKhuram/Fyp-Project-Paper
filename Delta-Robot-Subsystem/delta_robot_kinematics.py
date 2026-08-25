import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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


class DeltaVisualizer:
    """Builds the figures for the paper, dumps the underlying numbers to
    CSV alongside each PNG so nothing has to be re-typed by hand later."""

    def __init__(self, kinematics):
        self.kin = kinematics

    def _sample_workspace(self, res=20):
        rx, ry, rz = [], [], []
        max_r = 0.0
        for z in np.linspace(-250, -50, res):
            for x in np.linspace(-250, 250, res):
                for y in np.linspace(-250, 250, res):
                    if all(a is not None for a in self.kin.inverse(x, y, z)):
                        rx.append(x); ry.append(y); rz.append(z)
                        max_r = max(max_r, np.sqrt(x**2 + y**2))
        return rx, ry, rz, max_r

    def plot_3d_workspace(self, res=20):
        rx, ry, rz, max_r = self._sample_workspace(res)

        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(rx, ry, rz, c=rz, s=2, alpha=0.2, cmap='viridis')
        theta = np.linspace(0, 2 * np.pi, 100)
        ax.plot(max_r * np.cos(theta), max_r * np.sin(theta), -200, 'r--',
                label=f'Boundary (r={max_r:.1f}mm)')
        ax.set_title("Figure 2a: 3D Reachable Workspace Envelope (limits + collision applied)")
        ax.legend()
        plt.savefig(cfg.results_path("fig2a_workspace_3d.png"), dpi=300)
        plt.close()

        with open(cfg.results_path("workspace_summary.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            w.writerow(["reachable_points", len(rx)])
            w.writerow(["max_radius_mm", round(max_r, 2)])

        return max_r

    def plot_workspace_projections(self, res=20):
        rx, ry, rz, _ = self._sample_workspace(res)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.scatter(rx, ry, s=1, c=rz, alpha=0.1)
        ax1.set_title("Top View (XY Projection)")
        ax2.scatter(rx, rz, s=1, c=ry, alpha=0.1)
        ax2.set_title("Side View (XZ Projection)")
        plt.savefig(cfg.results_path("fig2b_workspace_projections.png"), dpi=300)
        plt.close()

    def plot_path_optimization(self, weeds):
        import itertools
        
        def path_length(order):
            pts = [[0.0, 0.0]] + list(order) + [[0.0, 0.0]]
            return sum(np.linalg.norm(np.array(pts[i + 1]) - np.array(pts[i]))
                       for i in range(len(pts) - 1))

        # Greedy nearest-neighbor path
        curr, unvisited = [0.0, 0.0], list(weeds)
        opt_path = [curr]
        while unvisited:
            nxt = min(unvisited, key=lambda w: np.linalg.norm(np.array(w) - np.array(curr)))
            opt_path.append(nxt); unvisited.remove(nxt); curr = nxt
        opt_path.append([0.0, 0.0])

        opt_len = path_length(opt_path[1:-1])
        rand_len = path_length(weeds)
        reduction = 100 * (1 - opt_len / rand_len)

        # Exact optimal TSP path
        home = [0.0, 0.0]
        exact_len = float('inf')
        exact_path = []
        for perm in itertools.permutations(weeds):
            tour = [home] + list(perm) + [home]
            dist = sum(np.linalg.norm(np.array(tour[k + 1]) - np.array(tour[k]))
                       for k in range(len(tour) - 1))
            if dist < exact_len:
                exact_len = dist
                exact_path = tour

        greedy_overhead = 100 * (opt_len - exact_len) / exact_len if exact_len > 0 else 0.0

        plt.figure(figsize=(7, 7))
        px, py = zip(*opt_path)
        plt.plot(px, py, 'b--', label=f'Greedy nearest-neighbor path ({opt_len:.1f} mm)', alpha=0.8)
        
        ex, ey = zip(*exact_path)
        plt.plot(ex, ey, 'g:', label=f'Exact optimal TSP tour ({exact_len:.1f} mm)', alpha=0.8)
        
        plt.scatter(*zip(*weeds), c='red', marker='x', s=100, label='Detected weeds')
        plt.plot(0, 0, 'go', label='Home')
        plt.title("Figure 3a: Weed-Removal Trajectory Optimization")
        plt.legend(); plt.grid(True); plt.axis('equal')
        plt.savefig(cfg.results_path("fig3a_path_optimization.png"), dpi=300)
        plt.close()

        with open(cfg.results_path("path_optimization_summary.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value_mm_or_pct"])
            w.writerow(["greedy_path_length_mm", round(opt_len, 1)])
            w.writerow(["unsorted_path_length_mm", round(rand_len, 1)])
            w.writerow(["exact_path_length_mm", round(exact_len, 1)])
            w.writerow(["reduction_pct", round(reduction, 1)])
            w.writerow(["greedy_overhead_pct", round(greedy_overhead, 1)])

        return opt_len, rand_len

    def plot_pid_response(self):
        Kp, Ki, Kd = cfg.PID_KP, cfg.PID_KI, cfg.PID_KD
        J, b = cfg.ACTUATOR_J, cfg.ACTUATOR_B
        target, dt = cfg.PID_TARGET_DEG, cfg.PID_DT_S
        time = np.arange(0, cfg.PID_SIM_DURATION_S, dt)
        curr, vel, integral, prev_err, history = 0.0, 0.0, 0.0, 0.0, []

        for _ in time:
            err = target - curr
            integral += err * dt
            torque = np.clip((Kp * err) + (Ki * integral) + (Kd * (err - prev_err) / dt),
                              -cfg.TORQUE_LIMIT_NM, cfg.TORQUE_LIMIT_NM)
            vel += ((torque - b * vel) / J) * dt
            curr += np.degrees(vel) * dt
            history.append(curr)
            prev_err = err

        plt.figure(figsize=(8, 5))
        plt.plot(time, history, label='Joint angle', lw=2)
        plt.axhline(target, color='r', ls='--', label=f'Setpoint ({target:.0f}°)')
        plt.title("Figure 3b: Joint Angle Step Response (MG996R, simulated)")
        plt.xlabel("Time (s)"); plt.ylabel("Angle (deg)"); plt.grid(True); plt.legend()
        plt.savefig(cfg.results_path("fig3b_pid_response.png"), dpi=300)
        plt.close()

        with open(cfg.results_path("pid_response_summary.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            w.writerow(["target_deg", target])
            w.writerow(["settled_angle_deg", round(history[-1], 2)])

        return history[-1]


if __name__ == '__main__':
    kin = DeltaKinematics()
    viz = DeltaVisualizer(kin)

    print(f"Results will be saved to: {cfg.RESULTS_DIR}\n")

    test_pts = [(20, 30, -150), (-90, 60, -160), (80, 20, -180), (10, -90, -200)]
    roundtrip_rows = []
    print("Round-trip check (IK -> FK should return the original point):")
    for pt in test_pts:
        angles = kin.inverse(*pt)
        back = kin.forward(*angles)
        err = np.linalg.norm(np.array(pt) - np.array(back)) if back else float('nan')
        print(f"  {pt} -> {tuple(round(b, 6) for b in back)}  error: {err:.2e}mm")
        roundtrip_rows.append([pt, back, err])

    with open(cfg.results_path("roundtrip_validation.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["target_xyz", "recovered_xyz", "error_mm"])
        for target, recovered, err in roundtrip_rows:
            w.writerow([target, tuple(round(c, 6) for c in recovered), f"{err:.3e}"])

    print("\nFigure 2a...")
    max_r = viz.plot_3d_workspace()
    print(f"Reachable radius: {max_r:.1f}mm")

    print("Figure 2b...")
    viz.plot_workspace_projections()

    print("Figure 3a...")
    weed_field = [(80, 20), (-90, 60), (100, -80), (-80, -90), (20, 100)]
    viz.plot_path_optimization(weed_field)

    print("Figure 3b...")
    final_angle = viz.plot_pid_response()
    print(f"Settled joint angle after 0.5s: {final_angle:.2f}° (target {cfg.PID_TARGET_DEG}°)")

    print(f"\nAll figures and CSV tables saved to: {cfg.RESULTS_DIR}")
