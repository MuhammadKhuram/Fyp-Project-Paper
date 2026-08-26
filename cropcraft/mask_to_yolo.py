import cv2
import numpy as np
import os
import glob

# Script is in the root of cropcraft folder
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')

CLASS_CROP = 0
CLASS_WEED = 1
KERNEL_SIZE = 45 

def process_masks():
    # Recursively find all masks in every sub-folder inside 'output/'
    search_pattern = os.path.join(OUTPUT_DIR, '**', 'masks', '*.*')
    mask_files = glob.glob(search_pattern, recursive=True)
    
    # Filter out non-image files just in case
    mask_files = [f for f in mask_files if f.endswith(('.png', '.jpg', '.jpeg'))]
    
    if not mask_files:
        print(f"❌ No masks found inside the '{OUTPUT_DIR}' folder structure.")
        return

    print(f"🔍 Found {len(mask_files)} masks across all fields. Starting conversion...")

    for mask_path in mask_files:
        # Intelligently find where to save the label for this specific mask
        masks_folder = os.path.dirname(mask_path)
        render_folder = os.path.dirname(masks_folder)
        labels_folder = os.path.join(render_folder, 'labels')
        
        # Create labels folder if it doesn't exist for this specific field
        if not os.path.exists(labels_folder):
            os.makedirs(labels_folder)
            
        filename = os.path.splitext(os.path.basename(mask_path))[0]
        label_path = os.path.join(labels_folder, f"{filename}.txt")
        
        mask_img = cv2.imread(mask_path)
        if mask_img is None:
            continue
            
        img_h, img_w = mask_img.shape[:2]
        yolo_lines = []

        lower_green = np.array([0, 100, 0])
        upper_green = np.array([100, 255, 100])
        green_mask = cv2.inRange(mask_img, lower_green, upper_green)

        lower_red = np.array([0, 0, 100])
        upper_red = np.array([100, 100, 255])
        red_mask = cv2.inRange(mask_img, lower_red, upper_red)

        def extract_boxes(binary_mask, class_id):
            kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
            fused_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(fused_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) < 50:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                x_center = (x + (w / 2.0)) / img_w
                y_center = (y + (h / 2.0)) / img_h
                norm_w = w / img_w
                norm_h = h / img_h
                yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}")

        extract_boxes(green_mask, CLASS_CROP)
        extract_boxes(red_mask, CLASS_WEED)

        with open(label_path, 'w') as f:
            f.write('\n'.join(yolo_lines))

    print(f"✅ Conversion complete! .txt files saved to their respective field folders.")

if __name__ == '__main__':
    process_masks()