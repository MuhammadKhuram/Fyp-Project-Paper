import os
import random
import shutil
import glob

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')

TRAIN_DIR = os.path.join(SCRIPT_DIR, 'train')
VAL_DIR = os.path.join(SCRIPT_DIR, 'val')

def setup_directories():
    folders = [
        os.path.join(TRAIN_DIR, 'images'), os.path.join(TRAIN_DIR, 'labels'),
        os.path.join(VAL_DIR, 'images'), os.path.join(VAL_DIR, 'labels')
    ]
    for folder in folders:
        os.makedirs(folder, exist_ok=True)

def create_yaml():
    yaml_path = os.path.join(SCRIPT_DIR, 'data.yaml')
    yaml_content = """train: train/images
val: val/images

nc: 2
names: ['crop', 'weed']
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"📄 Created data.yaml")

def split_data(split_ratio=0.8):
    # Recursively find all images in the output folder
    search_pattern = os.path.join(OUTPUT_DIR, '**', 'images', '*.*')
    image_files = glob.glob(search_pattern, recursive=True)
    image_files = [f for f in image_files if f.endswith(('.png', '.jpg', '.jpeg'))]
    
    if not image_files:
        print("❌ No images found in the 'output' subfolders!")
        return

    random.seed(42)
    random.shuffle(image_files)

    split_index = int(len(image_files) * split_ratio)
    train_images = image_files[:split_index]
    val_images = image_files[split_index:]

    print(f"📦 Found {len(image_files)} total images across all fields.")
    print(f"   --> Sending {len(train_images)} to Training Set (80%)")
    print(f"   --> Sending {len(val_images)} to Validation Set (20%)")

    def copy_files(img_list, destination_folder):
        for img_path in img_list:
            # Extract the field name to prevent overwriting images with the same name
            # Path usually looks like: .../output/bean_field_01/render/images/image_001.jpg
            parts = os.path.normpath(img_path).split(os.sep)
            field_name = parts[-4] if len(parts) >= 4 else "field"
            
            original_filename = os.path.basename(img_path)
            # Create a unique filename: e.g., "bean_field_01_image_001.jpg"
            new_filename = f"{field_name}_{original_filename}"
            
            original_name_only = os.path.splitext(original_filename)[0]
            new_name_only = os.path.splitext(new_filename)[0]
            
            # Copy Image
            shutil.copy(img_path, os.path.join(destination_folder, 'images', new_filename))
            
            # Find and Copy corresponding Label
            images_dir = os.path.dirname(img_path)
            render_dir = os.path.dirname(images_dir)
            label_path = os.path.join(render_dir, 'labels', f"{original_name_only}.txt")
            
            if os.path.exists(label_path):
                shutil.copy(label_path, os.path.join(destination_folder, 'labels', f"{new_name_only}.txt"))

    print("⏳ Copying training files...")
    copy_files(train_images, TRAIN_DIR)
    
    print("⏳ Copying validation files...")
    copy_files(val_images, VAL_DIR)
    print("\n✅ Unified Dataset successfully split and formatted for YOLOv8!")

if __name__ == '__main__':
    setup_directories()
    create_yaml()
    split_data(split_ratio=0.8)