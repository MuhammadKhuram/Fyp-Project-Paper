import csv
import math
import matplotlib.pyplot as plt
import os

# =========================================================
# Points at the standalone Milestone B controller's results folder.
# If you generate Fig 6a from a Milestone D run instead (they all also log
# trajectory.csv), just repoint this to that controller's results folder.
CSV_PATH = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\WEBOT2.O\controllers\mecanum_trajectory_log\results\trajectory_log.csv"
OUTPUT_FOLDER = r"C:\Users\User\Documents\WEEDS\Fyp-Project-Paper\Navigation_Subsystem\Results"
# =========================================================

# --- Load logged trajectory ---
times, xs, ys, headings = [], [], [], []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["time"]))
        xs.append(float(row["x"]))
        ys.append(float(row["y"]))
        headings.append(float(row["heading"]))

# --- Regenerate the same planned waypoints (must match controller exactly) ---
def generate_boustrophedon(field_w=10.0, field_h=10.0, stripe_width=0.65, margin=0.5):
    waypoints = []
    x_start = -field_w / 2 + margin
    x_end = field_w / 2 - margin
    y_min = -field_h / 2 + margin
    y_max = field_h / 2 - margin
    num_stripes = int((field_h - 2 * margin) / stripe_width) + 1
    for i in range(num_stripes):
        y = min(y_min + i * stripe_width, y_max)
        if i % 2 == 0:
            waypoints.append((x_start, y))
            waypoints.append((x_end, y))
        else:
            waypoints.append((x_end, y))
            waypoints.append((x_start, y))
    return waypoints

waypoints = generate_boustrophedon()
wx = [w[0] for w in waypoints]
wy = [w[1] for w in waypoints]

# --- Downsample actual trajectory for plotting ---
DOWNSAMPLE = 5
xs_plot = xs[::DOWNSAMPLE]
ys_plot = ys[::DOWNSAMPLE]
headings_plot = headings[::DOWNSAMPLE]
times_plot = times[::DOWNSAMPLE]

# --- Summary numbers (computed before layout so they can go in the stats box) ---
path_length = sum(
    math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    for i in range(1, len(xs))
)
mission_time = times[-1] - times[0]

def point_to_segment_dist(px, py, ax_, ay_, bx_, by_):
    abx, aby = bx_ - ax_, by_ - ay_
    apx, apy = px - ax_, py - ay_
    seg_len_sq = abx**2 + aby**2
    t = 0.0 if seg_len_sq == 0 else max(0, min(1, (apx * abx + apy * aby) / seg_len_sq))
    closest_x, closest_y = ax_ + t * abx, ay_ + t * aby
    return math.hypot(px - closest_x, py - closest_y)

deviations = []
for px, py in zip(xs, ys):
    min_d = min(
        point_to_segment_dist(px, py, wx[i], wy[i], wx[i + 1], wy[i + 1])
        for i in range(len(waypoints) - 1)
    )
    deviations.append(min_d)
avg_deviation = sum(deviations) / len(deviations)

# --- Figure layout: plot on top, legend+stats panel below ---
fig, (ax, info_ax) = plt.subplots(
    2, 1, figsize=(9, 10.5), gridspec_kw={"height_ratios": [4, 1]}
)

# field boundary (10x10, centered at origin)
field = plt.Rectangle((-5, -5), 10, 10, fill=False, edgecolor="black", linewidth=1.5)
ax.add_patch(field)

# planned path
line_planned, = ax.plot(wx, wy, linestyle="--", marker="o", color="gray",
                         markersize=3, label="Planned waypoints")

# actual trajectory
line_actual, = ax.plot(xs_plot, ys_plot, linestyle="-", color="tab:blue",
                        linewidth=1.5, label="Actual trajectory (GPS)")

# heading arrows along the actual path (every Nth sample so it's not cluttered)
ARROW_EVERY = max(1, len(xs_plot) // 40)   # ~40 arrows total regardless of run length
arrow_len = 0.25
quiv = ax.quiver(
    xs_plot[::ARROW_EVERY], ys_plot[::ARROW_EVERY],
    [arrow_len * math.cos(h) for h in headings_plot[::ARROW_EVERY]],
    [arrow_len * math.sin(h) for h in headings_plot[::ARROW_EVERY]],
    color="tab:orange", angles="xy", scale_units="xy", scale=1,
    width=0.004, label="Heading direction"
)

# start and end markers
start_marker = ax.scatter([xs[0]], [ys[0]], color="green", marker="*",
                           s=250, zorder=5, label="Start")
end_marker = ax.scatter([xs[-1]], [ys[-1]], color="red", marker="X",
                         s=150, zorder=5, label="End")

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Boustrophedon Coverage: Planned vs. Actual Path")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)

# --- Legend in its own row, below the plot ---
handles = [line_planned, line_actual, quiv, start_marker, end_marker]
labels = [h.get_label() for h in handles]

info_ax.axis("off")
info_ax.legend(handles, labels, loc="upper center", ncol=3, frameon=True,
               bbox_to_anchor=(0.5, 1.8), fontsize=10)

# --- Summary stats box, under the legend ---
stats_text = (
    f"Total path length driven:  {path_length:.2f} m\n"
    f"Total mission time:  {mission_time:.2f} s\n"
    f"Average deviation from planned path:  {avg_deviation:.3f} m\n"
    f"Waypoints completed:  {len(waypoints)}/{len(waypoints)}\n"
    f"Samples logged:  {len(xs)}"
)
info_ax.text(0.5, 0.5, stats_text, transform=info_ax.transAxes,
             ha="center", va="center", fontsize=10,
             bbox=dict(boxstyle="round,pad=0.6", facecolor="whitesmoke", edgecolor="gray"))

fig.subplots_adjust(hspace=0.5)

os.makedirs(OUTPUT_FOLDER, exist_ok=True)
fig_path = os.path.join(OUTPUT_FOLDER, "figure_6a_trajectory.png")
fig.savefig(fig_path, dpi=300, bbox_inches="tight")
plt.show()

print(f"Total path length driven: {path_length:.2f} m")
print(f"Total mission time: {mission_time:.2f} s")
print(f"Average deviation from planned path: {avg_deviation:.3f} m")
print(f"Figure saved to: {fig_path}")
