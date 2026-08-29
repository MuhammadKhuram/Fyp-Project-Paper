import os
import shutil
import glob

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')
TRAIN_DIR = os.path.join(SCRIPT_DIR, 'train')
VAL_DIR = os.path.join(SCRIPT_DIR, 'val')

def setup_directories():
    for folder in [os.path.join(TRAIN_DIR, 'images'), os.path.join(TRAIN_DIR, 'labels'),
                   os.path.join(VAL_DIR, 'images'), os.path.join(VAL_DIR, 'labels')]:
        os.makedirs(folder, exist_ok=True)

def create_yaml():
    with open(os.path.join(SCRIPT_DIR, 'data.yaml'), 'w') as f:
        f.write("train: train/images\nval: val/images\n\nnc: 2\nnames: ['crop', 'weed']\n")

def split_by_field(val_field_names=["bean_field_03", "bean_field_04", "bean_field_05", "bean_field_06"]):
    search_pattern = os.path.join(OUTPUT_DIR, '**', 'images', '*.*')
    image_files = [f for f in glob.glob(search_pattern, recursive=True) if f.endswith(('.png', '.jpg'))]
    
    if not image_files:
        print("No images found.")
        return

    train_images, val_images = [], []
    
    for img in image_files:
        # Check if ANY of the validation field names are in the folder path
        if any(val_name in os.path.dirname(img) for val_name in val_field_names):
            val_images.append(img)
        else:
            train_images.append(img)

    # Safety check to ensure you didn't typo the folder names
    if len(val_images) == 0:
        print("WARNING: 0 validation images found. Check your val_field_names for typos!")
        return

    def copy_files(img_list, dest_folder):
        for img_path in img_list:
            parts = os.path.normpath(img_path).split(os.sep)
            field_name = parts[-4] if len(parts) >= 4 else "field"
            orig_filename = os.path.basename(img_path)
            new_filename = f"{field_name}_{orig_filename}"
            new_name_only = os.path.splitext(new_filename)[0]
            
            shutil.copy(img_path, os.path.join(dest_folder, 'images', new_filename))
            
            label_path = os.path.join(os.path.dirname(os.path.dirname(img_path)), 'labels', f"{os.path.splitext(orig_filename)[0]}.txt")
            if os.path.exists(label_path):
                shutil.copy(label_path, os.path.join(dest_folder, 'labels', f"{new_name_only}.txt"))

    copy_files(train_images, TRAIN_DIR)
    copy_files(val_images, VAL_DIR)
    print(f"Split complete. Train: {len(train_images)}, Val: {len(val_images)}")

if __name__ == '__main__':
    # Wipe old directories to prevent ghost files from previous bad splits
    for d in [TRAIN_DIR, VAL_DIR]:
        if os.path.exists(d):
            shutil.rmtree(d)

    setup_directories()
    create_yaml()
    
    # Define EXACTLY which fields you want to hold out for validation.
    # Four 100-frame fields will give you an ideal 400-image val set.
    fields_to_validate = [
        "bean_field_03", 
        "bean_field_04", 
        "bean_field_05", 
        "bean_field_06"
    ]
    
    split_by_field(val_field_names=fields_to_validate)