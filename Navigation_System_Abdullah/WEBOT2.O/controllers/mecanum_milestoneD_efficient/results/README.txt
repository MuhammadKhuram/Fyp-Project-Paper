After a full 28/28 run, this folder will contain:
  trajectory_log.csv                (columns: time, x, y, heading)
  localization_log.csv              (columns: time, gt_x, gt_y, noisy_gps_x, noisy_gps_y, est_x, est_y, in_blackout)
  row_switch_log_efficient.csv      (columns: mode, switch_index, start_time_s, end_time_s, switch_time_s, energy_estimate)

Not yet wired into plot_row_switch.py — that script currently only compares
crab vs turn. When you're ready to add the efficient-mode comparison, say so
and I'll extend the plotting script to a 3-way comparison.

IMPORTANT per our discussion: the real efficiency claim for this mode rests on
TOTAL mission time/energy across the whole run (compare trajectory_log.csv's
final timestamp across all three modes), not just this row-switch CSV, since
the win here comes from eliminating turns BETWEEN switches, not during them.
