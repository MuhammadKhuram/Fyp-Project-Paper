After a full 28/28 run, this folder will contain:
  trajectory_log.csv           (columns: time, x, y, heading)
  localization_log.csv         (columns: time, gt_x, gt_y, noisy_gps_x, noisy_gps_y, est_x, est_y, in_blackout)
  row_switch_log_crab.csv      (columns: mode, switch_index, start_time_s, end_time_s, switch_time_s, energy_estimate)

row_switch_log_crab.csv is one of the two files plot_row_switch.py reads.
