import numpy as np

import config as cfg

# Random test data, kept separate from the kinematics/control logic so we
# can regenerate or scale up scenarios without touching subsystem code.
# All the ranges below come from config.py now - used to have our own
# copies here and they drifted out of sync with the actual operating
# envelope once, not repeating that.
#
# Always pass a seed for anything that ends up in the paper. "Tested on
# random layouts" isn't reproducible. "Tested on layouts from seed=42" is.


def generate_robot_frame_layout(n_weeds, seed=None, z_fixed=None):
    """n_weeds random points directly in the delta's own frame, inside the
    operating envelope, with a minimum spacing between them."""
    rng = np.random.default_rng(seed)
    weeds = []
    attempts = 0
    while len(weeds) < n_weeds and attempts < n_weeds * 200:
        attempts += 1
        r = cfg.OPERATING_RADIUS * np.sqrt(rng.uniform(0, 1))
        theta = rng.uniform(0, 2 * np.pi)
        x, y = r * np.cos(theta), r * np.sin(theta)
        z = z_fixed if z_fixed is not None else rng.uniform(cfg.Z_MIN, cfg.Z_MAX)
        if all(np.hypot(x - wx, y - wy) >= cfg.MIN_WEED_SEPARATION for wx, wy, _ in weeds):
            weeds.append((x, y, z))
    if len(weeds) < n_weeds:
        raise RuntimeError(
            f"Could only place {len(weeds)}/{n_weeds} weeds with {cfg.MIN_WEED_SEPARATION}mm "
            f"separation - reduce n_weeds or the separation constant."
        )
    return weeds


def generate_camera_frame_detections(n_weeds, seed=None):
    """n_weeds random points in the CAMERA's frame, before the
    CameraInterface offset - exercises the full pipeline including the
    coordinate transform instead of skipping straight to robot-frame."""
    rng = np.random.default_rng(seed)
    weeds = []
    attempts = 0
    while len(weeds) < n_weeds and attempts < n_weeds * 200:
        attempts += 1
        x = rng.uniform(*cfg.CAM_X_RANGE)
        y = rng.uniform(*cfg.CAM_Y_RANGE)
        z = cfg.CAM_Z_NOMINAL + rng.uniform(-cfg.CAM_Z_NOISE, cfg.CAM_Z_NOISE)
        if all(np.hypot(x - wx, y - wy) >= cfg.MIN_WEED_SEPARATION for wx, wy, _ in weeds):
            weeds.append((x, y, z))
    if len(weeds) < n_weeds:
        raise RuntimeError(f"Could only place {len(weeds)}/{n_weeds} weeds - reduce n_weeds.")
    return weeds


def generate_test_batch(n_scenarios, weeds_per_scenario_range=(3, 8), seed=None):
    """A batch of independent scenarios, each with its own random weed
    count, for running the sensitivity analysis across many layouts
    instead of trusting one fixed example."""
    rng = np.random.default_rng(seed)
    batch = []
    for _ in range(n_scenarios):
        n = int(rng.integers(weeds_per_scenario_range[0], weeds_per_scenario_range[1] + 1))
        scenario_seed = int(rng.integers(0, 2**31 - 1))
        batch.append(generate_camera_frame_detections(n, seed=scenario_seed))
    return batch


if __name__ == '__main__':
    print("Robot-frame layout, seed=1:")
    for w in generate_robot_frame_layout(5, seed=1, z_fixed=-170):
        print(" ", tuple(round(v, 1) for v in w))

    print("\nCamera-frame detections, seed=1:")
    for w in generate_camera_frame_detections(5, seed=1):
        print(" ", tuple(round(v, 1) for v in w))

    print("\nTest batch of 3 scenarios, seed=7:")
    for i, scenario in enumerate(generate_test_batch(3, seed=7)):
        print(f" scenario {i}: {len(scenario)} weeds")
