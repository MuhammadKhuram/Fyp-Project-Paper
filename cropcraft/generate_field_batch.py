"""
Generates N distinct bean-field YAML configs for cropcraft, with randomized
crop spacing, weed composition/density, field noise, and camera/render settings —
for building a varied synthetic dataset rather than repeatedly rendering one field.

Usage:
    python generate_field_batch.py

Outputs to /home/muhammad-khuram-linux/Desktop/Fyp-Project-Paper/Fyp-Project-Paper/cropcraft/configs/bean_field_01.yaml ... bean_field_15.yaml
"""

import random
import yaml
import os

# ---- Master settings ----
NUM_CONFIGS = 15
FRAMES_PER_CONFIG = 100
OUTPUT_DIR = "/home/muhammad-khuram-linux/Desktop/Fyp-Project-Paper/Fyp-Project-Paper/cropcraft/configs"
BATCH_SEED = 42  # change this to get a different randomized batch

WEED_TYPES = ["portulaca", "polygonum", "taraxacum"]

os.makedirs(OUTPUT_DIR, exist_ok=True)
master_rand = random.Random(BATCH_SEED)


def rand_range(rand, lo, hi, digits=3):
    return round(rand.uniform(lo, hi), digits)


def build_config(idx: int, rand: random.Random) -> dict:
    name = f"bean_field_{idx:02d}"

    # --- bed parameters ---
    row_distance = rand_range(rand, 0.6, 0.80)
    plant_distance = rand_range(rand, 0.10, 0.40)
    rows_count = rand.choice([6])
    plant_height = rand_range(rand, 0.1, 0.25)
    plants_count = rand.choice([30,40, 50, 60, 70])
    bed_width = round(rows_count * row_distance + rand_range(rand, 0.0, 0.5), 2)
    orientation = rand.choice(["random", "random", "random", "aligned", "zero"])  # weighted toward random

    # --- field noise (realism) ---
    noise_position = rand_range(rand, 0.012, 0.025)
    noise_tilt = rand_range(rand, 0.04, 0.10)
    noise_missing = rand_range(rand, 0.10, 0.20)
    noise_scale = rand_range(rand, 0.10, 0.15)

    # --- weeds: pick 1-3 species present, each with independent params ---
    n_weeds = rand.choice([1, 1, 2, 2, 3])  # weighted toward 1-2 species per field
    chosen_weeds = rand.sample(WEED_TYPES, n_weeds)

    weeds_cfg = {}
    for w in chosen_weeds:
        weeds_cfg[w] = {
            "plant_type": w,
            "density": rand_range(rand, 3.0, 8.0, 1),
            "distance_min": rand_range(rand, 0.15, 0.40),
            "max_height": rand_range(rand, 0.02, 0.15),
            "noise_scale": rand_range(rand, 0.20, 0.50),
            "noise_offset": rand_range(rand, 0.10, 0.20),
        }

    # --- stones (optional, most fields have some) ---
    include_stones = rand.random() < 0.8
    stones_cfg = None
    if include_stones:
        stones_cfg = {
            "density": rand_range(rand, 15.0, 60.0, 1),
            "distance_min": rand_range(rand, 0.03, 0.08),
            "noise_scale": rand_range(rand, 0.20, 0.40),
            "noise_offset": rand_range(rand, 0.05, 0.25),
        }

    # --- camera / render ---
    cam_height = rand_range(rand, 0.9, 1.1)
    fov_deg = rand.choice([50.0, 60.0, 70.0])
    env_rotation_deg = rand_range(rand, 0.0, 360.0, 1)
    y_jitter = round(min(0.4, row_distance * 0.6), 3)

    field_seed = rand.getrandbits(32)

    field = {
        "headland_width": rand_range(rand, 2.0, 6.0, 1),
        "random_seed": field_seed,
        "beds": {
            "bed1": {
                "plant_type": "bean",
                "plant_height": plant_height,
                "height_tolerance_coeff": 0.15,
                "row_distance": row_distance,
                "rows_count": rows_count,
                "plants_count": plants_count,
                "plant_distance": plant_distance,
                "bed_width": bed_width,
                "orientation": orientation,
            }
        },
        "noise": {
            "position": noise_position,
            "tilt": noise_tilt,
            "missing": noise_missing,
            "scale": noise_scale,
        },
        "weeds": weeds_cfg,
    }
    if stones_cfg:
        field["stones"] = stones_cfg

    config = {
        "output_enabled": ["blender", "gazebo"],
        "output": {
            "blender": {"type": "blender_file", "filename": f"{name}.blend"},
            "gazebo": {"type": "gazebo_model", "name": name, "path": "."},
        },
        "field": field,
        "render": {
            "directory": "render",
            "frames": FRAMES_PER_CONFIG,
            "samples": 12,
            #"cycles_device": "GPU",
            "resolution_x": 640,
            "resolution_y": 480,
            "env_rotation_deg": env_rotation_deg,
            "camera": {
                "height": cam_height,
                "fov_deg": fov_deg,
                "roll_deg": 0.0,
                "pitch_deg": 0.0,
                "yaw_deg": 0.0,
                "y_jitter": y_jitter,
            },
            "label_colors": {
                "crop": [0, 255, 0],
                "weed": [255, 0, 0],
                "background": [0, 0, 0],
            },
        },
    }
    return name, config


def main():
    summary = []
    for i in range(1, NUM_CONFIGS + 1):
        name, config = build_config(i, master_rand)
        path = os.path.join(OUTPUT_DIR, f"{name}.yaml")
        with open(path, "w") as f:
            yaml.dump(config, f, sort_keys=False, default_flow_style=False)

        weeds = list(config["field"]["weeds"].keys())
        summary.append(
            f"{name}: rows={config['field']['beds']['bed1']['rows_count']} "
            f"row_dist={config['field']['beds']['bed1']['row_distance']} "
            f"weeds={weeds} cam_h={config['render']['camera']['height']}"
        )

    print(f"Generated {NUM_CONFIGS} configs in {OUTPUT_DIR}/\n")
    print("\n".join(summary))


if __name__ == "__main__":
    main()
