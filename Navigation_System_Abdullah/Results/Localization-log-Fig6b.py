import csv
import math
import matplotlib.pyplot as plt
import os

# =========================================================
CSV_PATH = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\WEBOT2.O\controllers\mecanum_localization\results\localization_log.csv"
OUTPUT_FOLDER = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\Results"
BLACKOUT_START_S = 100.0
BLACKOUT_END_S = 140.0
# =========================================================

times, gt_x, gt_y, gps_x, gps_y, est_x, est_y, blackout = [], [], [], [], [], [], [], []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["time"]))
        gt_x.append(float(row["gt_x"]))
        gt_y.append(float(row["gt_y"]))
        gps_x.append(float(row["noisy_gps_x"]))
        gps_y.append(float(row["noisy_gps_y"]))
        est_x.append(float(row["est_x"]))
        est_y.append(float(row["est_y"]))
        blackout.append(int(row["in_blackout"]))

# --- Error = distance between filtered estimate and ground truth ---
error = [math.hypot(est_x[i] - gt_x[i], est_y[i] - gt_y[i]) for i in range(len(times))]
raw_gps_error = [math.hypot(gps_x[i] - gt_x[i], gps_y[i] - gt_y[i]) for i in range(len(times))]

# --- Split by blackout for summary stats ---
normal_errors = [error[i] for i in range(len(times)) if not blackout[i]]
blackout_errors = [error[i] for i in range(len(times)) if blackout[i]]

mean_error_overall = sum(error) / len(error)
mean_error_normal = sum(normal_errors) / len(normal_errors) if normal_errors else float("nan")
mean_error_blackout = sum(blackout_errors) / len(blackout_errors) if blackout_errors else float("nan")
max_error_blackout = max(blackout_errors) if blackout_errors else float("nan")

# --- Plot ---
fig, (ax, info_ax) = plt.subplots(
    2, 1, figsize=(9, 9.5), gridspec_kw={"height_ratios": [3, 1.1]}
)

ax.plot(times, error, color="tab:blue", linewidth=1.3, label="Filter estimate error")
ax.plot(times, raw_gps_error, color="lightcoral", linewidth=0.8, alpha=0.6, label="Raw noisy GPS error")

ax.axvspan(BLACKOUT_START_S, BLACKOUT_END_S, color="gray", alpha=0.25, label="GPS blackout window")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Localization error (m)")
ax.set_title("Localization Error vs. Time (Filtered Estimate vs. Ground Truth)")
ax.grid(True, alpha=0.3)

handles, labels = ax.get_legend_handles_labels()
info_ax.axis("off")
info_ax.legend(handles, labels, loc="upper center", ncol=1, frameon=True,
               bbox_to_anchor=(0.5, 1.0), fontsize=10)

stats_text = (
    f"Mean error (overall):  {mean_error_overall:.3f} m\n"
    f"Mean error (GPS active):  {mean_error_normal:.3f} m\n"
    f"Mean error (during blackout):  {mean_error_blackout:.3f} m\n"
    f"Max error (during blackout):  {max_error_blackout:.3f} m\n"
    f"Blackout window: {BLACKOUT_START_S:.0f}s – {BLACKOUT_END_S:.0f}s"
)
info_ax.text(0.5, 0.1, stats_text, transform=info_ax.transAxes,
             ha="center", va="center", fontsize=10,
             bbox=dict(boxstyle="round,pad=0.6", facecolor="whitesmoke", edgecolor="gray"))

fig.subplots_adjust(hspace=0.5)

os.makedirs(OUTPUT_FOLDER, exist_ok=True)
fig_path = os.path.join(OUTPUT_FOLDER, "figure_6b_localization_error.png")
fig.savefig(fig_path, dpi=300, bbox_inches="tight")
plt.show()

print(f"Mean error overall: {mean_error_overall:.3f} m")
print(f"Mean error (GPS active): {mean_error_normal:.3f} m")
print(f"Mean error (blackout): {mean_error_blackout:.3f} m")
print(f"Max error (blackout): {max_error_blackout:.3f} m")
print(f"Figure saved to: {fig_path}")
