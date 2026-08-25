"""
Milestone D plots: crab-walk vs turn-in-place vs efficient row switching.

Produces FOUR figures:
  1. figure_6c1_crab_individual.png       - per-event switch time & energy, crab mode only
  2. figure_6c2_turn_individual.png       - per-event switch time & energy, turn mode only
  3. figure_6c3_efficient_individual.png  - per-event switch time & energy, efficient mode only
  4. figure_6c4_comparison.png            - crab vs turn vs efficient, mean +/- std, both metrics

Each figure has the plot(s) on top, a bordered legend panel, and a bordered
stats panel stacked underneath (same visual style as Figures 6a/6b).

NOTE ON SCOPE: this script compares per-switch crab-strafe windows only.
It does NOT capture the efficient mode's real advantage (eliminating the
large in-row heading-correction turns between switches) -- that requires
a separate total-mission-time/energy comparison built from each mode's
full trajectory log, once those exist.

Edit the CONFIG paths below, then run:
    python plot_row_switch.py
"""

import csv
import os
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIG: update these paths to match your machine ---
CRAB_CSV_PATH = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\WEBOT2.O\controllers\mecanum_milestoneD_crab\results\row_switch_log_crab.csv"
TURN_CSV_PATH = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\WEBOT2.O\controllers\mecanum_milestoneD_turn\results\row_switch_log_turn.csv"
EFFICIENT_CSV_PATH = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\WEBOT2.O\controllers\mecanum_milestoneD_efficient\results\row_switch_log_efficient.csv"
OUTPUT_DIR = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\Results\Crab vs Turn"

# --- Color scheme (Okabe-Ito colorblind-safe palette) ---
CRAB_COLOR = "#E69F00"        # amber
TURN_COLOR = "#009E73"        # teal
EFFICIENT_COLOR = "#0072B2"   # blue (third Okabe-Ito color, colorblind-safe alongside amber/teal)
CRAB_COLOR_DARK = "#9C6B00"
TURN_COLOR_DARK = "#00694D"
EFFICIENT_COLOR_DARK = "#004C78"
GRID_COLOR = "#CCCCCC"


def load_row_switch_log(path):
    """Returns (switch_index array, switch_time_s array, energy_estimate array)."""
    indices, times, energies = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            indices.append(int(row["switch_index"]))
            times.append(float(row["switch_time_s"]))
            energies.append(float(row["energy_estimate"]))
    return np.array(indices), np.array(times), np.array(energies)


def style_panel(ax):
    ax.axis("off")
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_edgecolor("gray")
        spine.set_linewidth(0.8)


def plot_individual(mode_name, color, color_dark, indices, times, energies, output_path):
    n = len(indices)
    time_mean, time_std = times.mean(), times.std()
    energy_mean, energy_std = energies.mean(), energies.std()

    fig = plt.figure(figsize=(10.5, 11.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[3, 0.5, 0.75], hspace=0.5, wspace=0.35)

    ax_time = fig.add_subplot(gs[0, 0])
    ax_energy = fig.add_subplot(gs[0, 1])
    legend_ax = fig.add_subplot(gs[1, :])
    stats_ax = fig.add_subplot(gs[2, :])

    ax_time.bar(indices, times, color=color, edgecolor=color_dark, linewidth=0.8, width=0.65)
    ax_time.axhline(time_mean, color=color_dark, linestyle="--", linewidth=1.2)
    ax_time.set_xlabel("Row-switch event index")
    ax_time.set_ylabel("Switch time (s)")
    ax_time.set_title("Switch duration per event", fontsize=11)
    ax_time.set_xticks(indices)
    ax_time.grid(axis="y", linestyle="--", color=GRID_COLOR, alpha=0.7)

    ax_energy.bar(indices, energies, color=color, edgecolor=color_dark, linewidth=0.8, width=0.65)
    ax_energy.axhline(energy_mean, color=color_dark, linestyle="--", linewidth=1.2)
    ax_energy.set_xlabel("Row-switch event index")
    ax_energy.set_ylabel("Energy estimate (a.u.)")
    ax_energy.set_title("Energy proxy per event", fontsize=11)
    ax_energy.set_xticks(indices)
    ax_energy.grid(axis="y", linestyle="--", color=GRID_COLOR, alpha=0.7)

    fig.suptitle(f"Milestone D: {mode_name} Mode — Row-Switch Events (n={n})", fontsize=13, y=0.97)

    style_panel(legend_ax)
    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor=color, edgecolor=color_dark, linewidth=0.8),
        plt.Line2D([0], [0], color=color_dark, linestyle="--", linewidth=1.2),
    ]
    legend_ax.legend(
        handles, [f"{mode_name} mode, per-event value", "Mean across events"],
        loc="center", ncol=2, frameon=True, fontsize=10, bbox_to_anchor=(0.5, 0.5),
    )

    style_panel(stats_ax)
    stats_text = (
        f"Switch time:  mean = {time_mean:.3f} s   std = {time_std:.3f} s   "
        f"min = {times.min():.3f} s   max = {times.max():.3f} s\n"
        f"Energy proxy: mean = {energy_mean:.3f}   std = {energy_std:.3f}   "
        f"min = {energies.min():.3f}   max = {energies.max():.3f}\n"
        f"Events: n = {n} (single deterministic run, all in-run row-switch transitions)"
    )
    stats_ax.text(0.5, 0.5, stats_text, ha="center", va="center", fontsize=9.5,
                  family="monospace", transform=stats_ax.transAxes)

    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"Saved {output_path}")
    return time_mean, time_std, energy_mean, energy_std


def plot_comparison_three_way(
    crab_times, crab_energy, turn_times, turn_energy, efficient_times, efficient_energy, output_path
):
    modes = ["Crab", "Turn", "Efficient"]
    colors = [CRAB_COLOR, TURN_COLOR, EFFICIENT_COLOR]
    colors_dark = [CRAB_COLOR_DARK, TURN_COLOR_DARK, EFFICIENT_COLOR_DARK]

    n_vals = [len(crab_times), len(turn_times), len(efficient_times)]
    time_means = [crab_times.mean(), turn_times.mean(), efficient_times.mean()]
    time_stds = [crab_times.std(), turn_times.std(), efficient_times.std()]
    energy_means = [crab_energy.mean(), turn_energy.mean(), efficient_energy.mean()]
    energy_stds = [crab_energy.std(), turn_energy.std(), efficient_energy.std()]

    fig = plt.figure(figsize=(10.5, 11.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[3, 0.55, 0.95], hspace=0.5, wspace=0.35)

    ax_time = fig.add_subplot(gs[0, 0])
    ax_energy = fig.add_subplot(gs[0, 1])
    legend_ax = fig.add_subplot(gs[1, :])
    stats_ax = fig.add_subplot(gs[2, :])

    bars_time = ax_time.bar(
        modes, time_means, yerr=time_stds, capsize=6,
        color=colors, edgecolor=colors_dark, linewidth=1.0,
    )
    ax_time.set_ylabel("Switch time (s)")
    ax_time.set_title("Row-switch duration", fontsize=11)
    ax_time.grid(axis="y", linestyle="--", color=GRID_COLOR, alpha=0.7)
    for rect, val, sd in zip(bars_time, time_means, time_stds):
        ax_time.text(rect.get_x() + rect.get_width() / 2, rect.get_height() + sd + 0.03,
                     f"{val:.2f}s", ha="center", va="bottom", fontsize=9)

    bars_energy = ax_energy.bar(
        modes, energy_means, yerr=energy_stds, capsize=6,
        color=colors, edgecolor=colors_dark, linewidth=1.0,
    )
    ax_energy.set_ylabel("Energy estimate (a.u.)")
    ax_energy.set_title("Row-switch energy proxy", fontsize=11)
    ax_energy.grid(axis="y", linestyle="--", color=GRID_COLOR, alpha=0.7)
    for rect, val, sd in zip(bars_energy, energy_means, energy_stds):
        ax_energy.text(rect.get_x() + rect.get_width() / 2, rect.get_height() + sd + 0.03,
                       f"{val:.2f}", ha="center", va="bottom", fontsize=9)

    fig.suptitle("Milestone D: Crab vs Turn vs Efficient Row Switching", fontsize=13, y=0.97)

    style_panel(legend_ax)
    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=cd, linewidth=1.0)
        for c, cd in zip(colors, colors_dark)
    ]
    legend_ax.legend(
        handles, [f"{m} mode (n={n} events)" for m, n in zip(modes, n_vals)],
        loc="center", ncol=3, frameon=True, fontsize=10, bbox_to_anchor=(0.5, 0.5),
    )

    style_panel(stats_ax)
    stats_text = (
        f"Switch time:   crab = {time_means[0]:.3f} ± {time_stds[0]:.3f} s   |   "
        f"turn = {time_means[1]:.3f} ± {time_stds[1]:.3f} s   |   "
        f"efficient = {time_means[2]:.3f} ± {time_stds[2]:.3f} s\n"
        f"Energy proxy:  crab = {energy_means[0]:.3f} ± {energy_stds[0]:.3f}   |   "
        f"turn = {energy_means[1]:.3f} ± {energy_stds[1]:.3f}   |   "
        f"efficient = {energy_means[2]:.3f} ± {energy_stds[2]:.3f}\n"
        f"Events per mode: crab n={n_vals[0]}, turn n={n_vals[1]}, efficient n={n_vals[2]}  "
        f"(all in-run row-switch events, single deterministic run per mode)\n"
        f"NOTE: this compares crab-strafe windows only. Efficient mode's main advantage\n"
        f"(fewer/smaller in-row turns) is NOT captured here -- see total mission-time comparison."
    )
    stats_ax.text(0.5, 0.5, stats_text, ha="center", va="center", fontsize=8.5,
                  family="monospace", transform=stats_ax.transAxes)

    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"Saved {output_path}")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    crab_idx, crab_times, crab_energy = load_row_switch_log(CRAB_CSV_PATH)
    turn_idx, turn_times, turn_energy = load_row_switch_log(TURN_CSV_PATH)
    eff_idx, eff_times, eff_energy = load_row_switch_log(EFFICIENT_CSV_PATH)

    plot_individual(
        "Crab", CRAB_COLOR, CRAB_COLOR_DARK, crab_idx, crab_times, crab_energy,
        os.path.join(OUTPUT_DIR, "figure_6c1_crab_individual.png"),
    )
    plot_individual(
        "Turn", TURN_COLOR, TURN_COLOR_DARK, turn_idx, turn_times, turn_energy,
        os.path.join(OUTPUT_DIR, "figure_6c2_turn_individual.png"),
    )
    plot_individual(
        "Efficient", EFFICIENT_COLOR, EFFICIENT_COLOR_DARK, eff_idx, eff_times, eff_energy,
        os.path.join(OUTPUT_DIR, "figure_6c3_efficient_individual.png"),
    )
    plot_comparison_three_way(
        crab_times, crab_energy, turn_times, turn_energy, eff_times, eff_energy,
        os.path.join(OUTPUT_DIR, "figure_6c4_comparison.png"),
    )

    plt.show()


if __name__ == "__main__":
    main()