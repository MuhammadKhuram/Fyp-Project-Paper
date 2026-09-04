"""
Generates a grayscale weed-density mask (for CropCraft's scattering_mode: image)
that permits weeds in side bands alongside crop rows, while excluding the full
longitudinal crop-row corridor.

Usage: edit the CONFIG block below to match your YAML's field/bed values, then run:
    python generate_weed_mask.py
"""

from PIL import Image, ImageDraw
from pathlib import Path
import random

# ---- CONFIG: copy these values straight from your YAML ----
bed_width = 3.0
rows_count = 6
row_distance = 0.75
plants_count = 60
plant_distance = 0.15
scattering_extra_width = 1.0   # field.scattering_extra_width (default 1.0 if not set)
bed_offset_y = 0.0             # bed.offset[1], default 0.0

row_clearance = 0.06           # keep weeds clear of the crop stems
side_band_width = 0.14         # weeds stay within 0.20 m of a crop row
noise_strength = 0.28          # density variation inside the permitted bands
noise_seed = 123456789          # deterministic mask variation
px_per_meter = 100             # image resolution
output_path = Path(__file__).resolve().parent / "examples/row_side_bands_mask.png"
# -------------------------------------------------------------

length = (plants_count - 1) * plant_distance
width = bed_width  # single bed, beds_count=1

row_offset = (bed_width - (rows_count - 1) * row_distance) / 2.0
row_ys = [bed_offset_y + row_offset + i * row_distance for i in range(rows_count)]
print("Row y-positions (m):", row_ys)

x_min, x_max = -scattering_extra_width, length + scattering_extra_width
y_min, y_max = -scattering_extra_width, width + scattering_extra_width

img_w = int((x_max - x_min) * px_per_meter)
img_h = int((y_max - y_min) * px_per_meter)

# white = full weed density, black = no weeds
img = Image.new("L", (img_w, img_h), color=0)
draw = ImageDraw.Draw(img)
noise = random.Random(noise_seed)

def y_to_px(y_real):
    # V=0 at bottom (Blender Generated-coord convention) -> flip vertically for PIL (row 0 = top)
    frac = (y_real - y_min) / (y_max - y_min)
    return img_h - int(frac * img_h)

for row_y in row_ys:
    for y_low, y_high in (
        (row_y - row_clearance - side_band_width, row_y - row_clearance),
        (row_y + row_clearance, row_y + row_clearance + side_band_width),
    ):
        y_top = y_to_px(y_high)
        y_bot = y_to_px(y_low)
        draw.rectangle([0, y_top, img_w, y_bot], fill=255)

# Vary density only where weeds are already allowed. This adds local gaps and
# clusters without creating weeds away from the crop rows.
for y in range(img_h):
    for x in range(img_w):
        value = img.getpixel((x, y))
        if value:
            variation = 1.0 - noise_strength * noise.random()
            img.putpixel((x, y), int(value * variation))

img.save(output_path)
print(f"Saved {output_path} ({img_w}x{img_h}px)")
