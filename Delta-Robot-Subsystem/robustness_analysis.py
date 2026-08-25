import csv
import itertools
import numpy as np
import matplotlib.pyplot as plt

import config as cfg
from delta_robot_kinematics import DeltaKinematics
from delta_robot_integration import CameraInterface, BaseCoupling, run_pipeline
from test_data_generator import generate_test_batch

N_SCENARIOS = 25
SEED = 100  # fixed so the numbers in the paper are reproducible


def _path_length(points):
    pts = [(0.0, 0.0)] + list(points) + [(0.0, 0.0)]
    return sum(np.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def solve_tsp_exact(points):
    """
    Finds the exact shortest path starting from (0,0), visiting all points,
    and returning to (0,0).
    """
    if not points:
        return 0.0
    home = (0.0, 0.0)
    best_len = float('inf')
    
    # Brute force search is feasible for N <= 8 (8! = 40,320 permutations)
    for p in itertools.permutations(points):
        path = [home] + list(p) + [home]
        dist = sum(np.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))
        if dist < best_len:
            best_len = dist
    return best_len


def run_scenario(kin, cam, coupling, detections):
    result = run_pipeline(kin, cam, coupling, detections)

    # run_pipeline already greedy-orders the reachable points into "trajectory"
    robot_pts = cam.batch_to_robot_frame(detections)
    reachable_xy = [(p[0], p[1]) for p in robot_pts if coupling.check_reachable(*p)]
    
    unsorted_len = _path_length(reachable_xy)
    greedy_len = _path_length([(x, y) for x, y, _ in result["trajectory"]])
    exact_len = solve_tsp_exact(reachable_xy)
    
    reduction_pct = 100 * (1 - greedy_len / unsorted_len) if unsorted_len > 0 else 0.0
    overhead_pct = 100 * (greedy_len - exact_len) / exact_len if exact_len > 0 else 0.0

    return (result["cycle_time_s"], reduction_pct, overhead_pct, 
            result["n_reachable"], result["n_unreachable"],
            unsorted_len, greedy_len, exact_len)


def main():
    kin = DeltaKinematics()
    cam = CameraInterface()
    coupling = BaseCoupling(kin)

    batch = generate_test_batch(N_SCENARIOS, seed=SEED)

    rows = []
    unsorted_lens = []
    greedy_lens = []
    exact_lens = []
    
    for i, scenario in enumerate(batch):
        (cycle_time, reduction_pct, overhead_pct, n_reach, n_unreach,
         unsorted_len, greedy_len, exact_len) = run_scenario(kin, cam, coupling, scenario)
        
        rows.append({
            "scenario": i,
            "n_weeds": len(scenario),
            "n_reachable": n_reach,
            "n_unreachable": n_unreach,
            "cycle_time_s": round(cycle_time, 2),
            "path_reduction_pct": round(reduction_pct, 1),
            "path_overhead_pct": round(overhead_pct, 1),
        })
        unsorted_lens.append(unsorted_len)
        greedy_lens.append(greedy_len)
        exact_lens.append(exact_len)

    cycle_times = np.array([r["cycle_time_s"] for r in rows])
    reductions = np.array([r["path_reduction_pct"] for r in rows])
    overheads = np.array([r["path_overhead_pct"] for r in rows])

    print(f"Ran {N_SCENARIOS} random scenarios (seed={SEED}):\n")
    print(f"Cycle time (s)      - mean: {cycle_times.mean():.2f}  std: {cycle_times.std():.2f}  "
          f"min: {cycle_times.min():.2f}  max: {cycle_times.max():.2f}")
    print(f"Path reduction %    - mean: {reductions.mean():.1f}  std: {reductions.std():.1f}  "
          f"min: {reductions.min():.1f}  max: {reductions.max():.1f}")
    print(f"Greedy overhead %   - mean: {overheads.mean():.1f}  std: {overheads.std():.1f}  "
          f"min: {overheads.min():.1f}  max: {overheads.max():.1f}")

    out_path = cfg.results_path("robustness_analysis.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
        w.writerow({})
        w.writerow({"scenario": "mean", 
                    "cycle_time_s": round(cycle_times.mean(), 3),
                    "path_reduction_pct": round(reductions.mean(), 2),
                    "path_overhead_pct": round(overheads.mean(), 2)})
        w.writerow({"scenario": "std", 
                    "cycle_time_s": round(cycle_times.std(), 3),
                    "path_reduction_pct": round(reductions.std(), 2),
                    "path_overhead_pct": round(overheads.std(), 2)})
        w.writerow({"scenario": "min", 
                    "cycle_time_s": round(cycle_times.min(), 3),
                    "path_reduction_pct": round(reductions.min(), 2),
                    "path_overhead_pct": round(overheads.min(), 2)})
        w.writerow({"scenario": "max", 
                    "cycle_time_s": round(cycle_times.max(), 3),
                    "path_reduction_pct": round(reductions.max(), 2),
                    "path_overhead_pct": round(overheads.max(), 2)})

    print(f"\nSaved per-scenario results and summary stats to: {out_path}")

    # Plot results
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    scenario_idx = np.arange(N_SCENARIOS)

    # Left plot: Cycle times
    ax1.bar(scenario_idx, cycle_times, color='steelblue', alpha=0.85)
    ax1.axhline(cycle_times.mean(), color='crimson', ls='--', lw=1.5,
               label=f'mean = {cycle_times.mean():.2f}s')
    ax1.set_title("Cycle Time per Scenario", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Scenario Index")
    ax1.set_ylabel("Cycle Time (s)")
    ax1.set_xticks(scenario_idx)
    ax1.set_xticklabels(scenario_idx, rotation=45, fontsize=8)
    ax1.grid(axis='y', ls=':', alpha=0.6)
    ax1.legend()

    # Right plot: Path length comparison (Grouped bar chart)
    width = 0.25
    ax2.bar(scenario_idx - width, unsorted_lens, width, label='Unsorted Order', color='#e06666')
    ax2.bar(scenario_idx, greedy_lens, width, label='Greedy NN', color='#ffd966')
    ax2.bar(scenario_idx + width, exact_lens, width, label='Exact Optimal (TSP)', color='#93c47d')
    ax2.set_title("Path Length Comparison", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Scenario Index")
    ax2.set_ylabel("Path Length (mm)")
    ax2.set_xticks(scenario_idx)
    ax2.set_xticklabels(scenario_idx, rotation=45, fontsize=8)
    ax2.grid(axis='y', ls=':', alpha=0.6)
    ax2.legend()

    plt.tight_layout()
    plt.savefig(cfg.results_path("fig4a_robustness_analysis.png"), dpi=300)
    plt.close()
    print(f"Saved figure to: {cfg.results_path('fig4a_robustness_analysis.png')}")


if __name__ == '__main__':
    main()
