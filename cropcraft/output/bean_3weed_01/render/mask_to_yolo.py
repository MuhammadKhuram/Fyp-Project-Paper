import cv2
import numpy as np
import os
import glob

# Get the exact folder where this Python script is saved
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Point to the masks and labels folders right next to the script
MASKS_DIR = os.path.join(SCRIPT_DIR, 'masks')
LABELS_DIR = os.path.join(SCRIPT_DIR, 'labels')

# Class IDs: 0 for Crop (Green), 1 for Weed (Red)
CLASS_CROP = 0
CLASS_WEED = 1

# ==========================================
# THE FUSION DIAL (Morphological Kernel Size)
# If the leaves are still separate, increase this to 55 or 65. 
# If the box starts grabbing two completely different plants, lower it to 25.
KERNEL_SIZE = 45 
# ==========================================

def process_masks():
    mask_files = glob.glob(os.path.join(MASKS_DIR, '*.*'))
    
    if not mask_files:
        print(f"❌ No masks found in:\n{MASKS_DIR}\nPlease check your folder name!")
        return

    print(f"🔍 Found {len(mask_files)} masks. Starting conversion...")

    # Create the label folder if it doesn't exist yet
    if not os.path.exists(LABELS_DIR):
        os.makedirs(LABELS_DIR)

    for mask_path in mask_files:
        filename = os.path.splitext(os.path.basename(mask_path))[0]
        label_path = os.path.join(LABELS_DIR, f"{filename}.txt")
        
        mask_img = cv2.imread(mask_path)
        if mask_img is None:
            continue
            
        img_h, img_w = mask_img.shape[:2]
        yolo_lines = []

        # Isolate GREEN pixels (Crops)
        lower_green = np.array([0, 100, 0])
        upper_green = np.array([100, 255, 100])
        green_mask = cv2.inRange(mask_img, lower_green, upper_green)

        # Isolate RED pixels (Weeds)
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([100, 100, 255])
        red_mask = cv2.inRange(mask_img, lower_red, upper_red)

        # Helper function to fuse leaves and convert to YOLO format
        def extract_boxes(binary_mask, class_id):
            # THE MORPHOLOGICAL TRICK:
            # 1. Create the "fusion block"
            kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
            # 2. Apply MORPH_CLOSE to bridge the gaps between leaves
            fused_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours on the newly fused mask
            contours, _ = cv2.findContours(fused_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) < 50: # Ignore tiny noise
                    continue
                    
                x, y, w, h = cv2.boundingRect(contour)
                
                # Convert to normalized YOLO math
                x_center = (x + (w / 2.0)) / img_w
                y_center = (y + (h / 2.0)) / img_h
                norm_w = w / img_w
                norm_h = h / img_h
                
                yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}")

        # Extract boxes
        extract_boxes(green_mask, CLASS_CROP)
        extract_boxes(red_mask, CLASS_WEED)

        # Save to .txt file
        with open(label_path, 'w') as f:
            f.write('\n'.join(yolo_lines))

    print(f"✅ Conversion complete! {len(mask_files)} .txt files saved to the 'labels' folder.")

if __name__ == '__main__':
    process_masks()