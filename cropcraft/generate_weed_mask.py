"""
Generates a grayscale weed-density mask (for CropCraft's scattering_mode: image)
that excludes weeds from directly under crop rows, based on the same row-position
formula used in core/beds.py.

Usage: edit the CONFIG block below to match your YAML's field/bed values, then run:
    python generate_weed_mask.py
"""

from PIL import Image, ImageDraw

# ---- CONFIG: copy these values straight from your YAML ----
bed_width = 3.0
rows_count = 6
row_distance = 0.75
plants_count = 60
plant_distance = 0.15
scattering_extra_width = 1.0   # field.scattering_extra_width (default 1.0 if not set)
bed_offset_y = 0.0             # bed.offset[1], default 0.0

band_half_width = 0.15         # meters either side of row center to exclude (tune to canopy width)
px_per_meter = 100             # image resolution
output_path = "row_exclusion_mask.png"
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
img = Image.new("L", (img_w, img_h), color=255)
draw = ImageDraw.Draw(img)

def y_to_px(y_real):
    # V=0 at bottom (Blender Generated-coord convention) -> flip vertically for PIL (row 0 = top)
    frac = (y_real - y_min) / (y_max - y_min)
    return img_h - int(frac * img_h)

for row_y in row_ys:
    y_top = y_to_px(row_y + band_half_width)
    y_bot = y_to_px(row_y - band_half_width)
    draw.rectangle([0, y_top, img_w, y_bot], fill=0)

img.save(output_path)
print(f"Saved {output_path} ({img_w}x{img_h}px)")
