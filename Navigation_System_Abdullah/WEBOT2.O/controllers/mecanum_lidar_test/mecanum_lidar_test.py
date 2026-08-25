"""
Milestone E - LiDAR sanity check (standalone, not the real controller)

Purpose: confirm the LiDAR device is found, enabled, and mounted the
way we expect BEFORE wiring it into navigation. This does NOT drive
the robot - it just sits still and prints range data every step.

How to use:
1. Put this file in its own folder: controllers/mecanum_lidar_test/mecanum_lidar_test.py
   (folder name must match filename exactly, per your project convention)
2. Set the mecanum_platform robot's controller field to "mecanum_lidar_test"
3. Place ONE known object directly in front of the robot at a known
   distance (e.g. a box 1.0m straight ahead along the chassis's +X axis)
4. Run the sim, watch the console output described below
5. Once confirmed, switch the controller field back before running
   the real Milestone E controller
"""

import math
from controller import Robot

TIME_STEP = 32
NUM_SECTORS = 15

robot = Robot()

lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)

h_res = lidar.getHorizontalResolution()
fov = lidar.getFov()
max_range = lidar.getMaxRange()

print(f"[LidarTest] horizontalResolution={h_res}, fov_rad={fov:.4f} "
      f"({math.degrees(fov):.1f} deg), maxRange={max_range}")

# Keep wheels at zero so the robot doesn't move during this test
for name in ("wheel_fl_motor", "wheel_fr_motor", "wheel_rl_motor", "wheel_rr_motor"):
    m = robot.getDevice(name)
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

step_count = 0

while robot.step(TIME_STEP) != -1:
    step_count += 1
    if step_count % 15 != 0:  # print roughly twice a second instead of every step
        continue

    range_image = lidar.getRangeImage()
    n = len(range_image)

    # Same binning the real controller uses
    rays_per_sector = max(1, n // NUM_SECTORS)
    sector_mins = []
    for s in range(NUM_SECTORS):
        start = s * rays_per_sector
        end = n if s == NUM_SECTORS - 1 else (s + 1) * rays_per_sector
        chunk = range_image[start:end]
        valid = [r for r in chunk if not math.isnan(r) and not math.isinf(r)]
        min_r = min(valid) if valid else max_range
        mid_index = (start + end - 1) / 2.0
        angle_deg = math.degrees((fov / 2.0) - (mid_index / (n - 1)) * fov)
        sector_mins.append((round(angle_deg, 1), round(min_r, 3)))

    # Raw endpoints too, so you can sanity-check index order directly
    first_ray = range_image[0]
    last_ray = range_image[-1]
    middle_ray = range_image[n // 2]

    print(f"[LidarTest] t={robot.getTime():.1f}s | "
          f"first_ray={first_ray:.3f} mid_ray={middle_ray:.3f} last_ray={last_ray:.3f}")
    print(f"[LidarTest] sectors (angle_deg, min_range_m): {sector_mins}")