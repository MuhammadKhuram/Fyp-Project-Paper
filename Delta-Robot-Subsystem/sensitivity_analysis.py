import csv
import numpy as np
import matplotlib.pyplot as plt

import config as cfg
from delta_robot_kinematics import DeltaKinematics
from delta_robot_integration import CameraInterface, BaseCoupling, run_pipeline
from test_data_generator import generate_test_batch

# Same pipeline as delta_robot_integration.py's __main__ block, just run
# across a batch of random scenarios instead of one fixed 5-weed layout.
# One example is an anecdote - this is what actually lets us report an
# average cycle time and a reduction figure with a spread on it instead
# of a single number that happened to come out favorably.

N_SCENARIOS = 25
SEED = 100  # fixed so the numbers in the paper are reproducible


def _path_length(points):
    pts = [(0.0, 0.0)] + list(points) + [(0.0, 0.0)]
    return sum(np.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def run_scenario(kin, cam, coupling, detections):
    result = run_pipeline(kin, cam, coupling, detections)

    # run_pipeline already greedy-orders the reachable points into
    # "trajectory" - path reduction compares that against the same
    # points left in whatever order they were detected in.
    robot_pts = cam.batch_to_robot_frame(detections)
    reachable_xy = [(p[0], p[1]) for p in robot_pts if coupling.check_reachable(*p)]
    unsorted_len = _path_length(reachable_xy)
    greedy_len = _path_length([(x, y) for x, y, _ in result["trajectory"]])
    reduction_pct = 100 * (1 - greedy_len / unsorted_len) if unsorted_len > 0 else 0.0

    return result["cycle_time_s"], reduction_pct, result["n_reachable"], result["n_unreachable"]


def main():
    kin = DeltaKinematics()
    cam = CameraInterface()
    coupling = BaseCoupling(kin)

    batch = generate_test_batch(N_SCENARIOS, seed=SEED)

    rows = []
    for i, scenario in enumerate(batch):
        cycle_time, reduction_pct, n_reach, n_unreach = run_scenario(kin, cam, coupling, scenario)
        rows.append({
            "scenario": i,
            "n_weeds": len(scenario),
            "n_reachable": n_reach,
            "n_unreachable": n_unreach,
            "cycle_time_s": round(cycle_time, 2),
            "path_reduction_pct": round(reduction_pct, 1),
        })

    cycle_times = np.array([r["cycle_time_s"] for r in rows])
    reductions = np.array([r["path_reduction_pct"] for r in rows])

    print(f"Ran {N_SCENARIOS} random scenarios (seed={SEED}):\n")
    print(f"Cycle time (s)   - mean: {cycle_times.mean():.2f}  std: {cycle_times.std():.2f}  "
          f"min: {cycle_times.min():.2f}  max: {cycle_times.max():.2f}")
    print(f"Path reduction % - mean: {reductions.mean():.1f}  std: {reductions.std():.1f}  "
          f"min: {reductions.min():.1f}  max: {reductions.max():.1f}")

    out_path = cfg.results_path("sensitivity_analysis.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
        w.writerow({})
        w.writerow({"scenario": "mean", "cycle_time_s": round(cycle_times.mean(), 3),
                    "path_reduction_pct": round(reductions.mean(), 2)})
        w.writerow({"scenario": "std", "cycle_time_s": round(cycle_times.std(), 3),
                    "path_reduction_pct": round(reductions.std(), 2)})
        w.writerow({"scenario": "min", "cycle_time_s": round(cycle_times.min(), 3),
                    "path_reduction_pct": round(reductions.min(), 2)})
        w.writerow({"scenario": "max", "cycle_time_s": round(cycle_times.max(), 3),
                    "path_reduction_pct": round(reductions.max(), 2)})

    print(f"\nSaved per-scenario results and summary stats to: {out_path}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    scenario_idx = np.arange(N_SCENARIOS)

    ax1.bar(scenario_idx, cycle_times, color='steelblue')
    ax1.axhline(cycle_times.mean(), color='red', ls='--', label=f'mean = {cycle_times.mean():.2f}s')
    ax1.set_title("Cycle Time per Scenario")
    ax1.set_xlabel("Scenario"); ax1.set_ylabel("Cycle time (s)")
    ax1.legend()

    ax2.bar(scenario_idx, reductions, color='seagreen')
    ax2.axhline(reductions.mean(), color='red', ls='--', label=f'mean = {reductions.mean():.1f}%')
    ax2.axhline(0, color='black', lw=0.8)
    ax2.set_title("Path Length Reduction per Scenario")
    ax2.set_xlabel("Scenario"); ax2.set_ylabel("Reduction vs. unsorted (%)")
    ax2.legend()

    plt.tight_layout()
    plt.savefig(cfg.results_path("fig4a_sensitivity_analysis.png"), dpi=300)
    plt.show()


if __name__ == '__main__':
    main()
