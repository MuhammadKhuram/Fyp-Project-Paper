import os
import random
import shutil
import glob

# Get the exact folder where this Python script is saved (your 'render' folder)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Original folders
IMAGES_DIR = os.path.join(SCRIPT_DIR, 'images')
LABELS_DIR = os.path.join(SCRIPT_DIR, 'labels')

# New YOLO folders
TRAIN_DIR = os.path.join(SCRIPT_DIR, 'train')
VAL_DIR = os.path.join(SCRIPT_DIR, 'val')

def setup_directories():
    """Create the train and val directories."""
    folders = [
        os.path.join(TRAIN_DIR, 'images'),
        os.path.join(TRAIN_DIR, 'labels'),
        os.path.join(VAL_DIR, 'images'),
        os.path.join(VAL_DIR, 'labels')
    ]
    for folder in folders:
        os.makedirs(folder, exist_ok=True)

def create_yaml():
    """Generate the data.yaml file automatically."""
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
    # Find all images
    image_files = glob.glob(os.path.join(IMAGES_DIR, '*.*'))
    
    if not image_files:
        print("❌ No images found in the 'images' folder!")
        return

    # Shuffle the images randomly
    random.seed(42) # Keeps the random shuffle consistent
    random.shuffle(image_files)

    # Calculate split index
    split_index = int(len(image_files) * split_ratio)
    train_images = image_files[:split_index]
    val_images = image_files[split_index:]

    print(f"📦 Found {len(image_files)} total images.")
    print(f"   --> Sending {len(train_images)} to Training Set (80%)")
    print(f"   --> Sending {len(val_images)} to Validation Set (20%)")

    # Helper function to copy files
    def copy_files(img_list, destination_folder):
        for img_path in img_list:
            filename = os.path.basename(img_path)
            name_only = os.path.splitext(filename)[0]
            
            # Copy Image
            shutil.copy(img_path, os.path.join(destination_folder, 'images', filename))
            
            # Copy Label (if it exists)
            label_path = os.path.join(LABELS_DIR, f"{name_only}.txt")
            if os.path.exists(label_path):
                shutil.copy(label_path, os.path.join(destination_folder, 'labels', f"{name_only}.txt"))

    # Execute copying
    print("⏳ Copying training files...")
    copy_files(train_images, TRAIN_DIR)
    
    print("⏳ Copying validation files...")
    copy_files(val_images, VAL_DIR)

    print("\n✅ Dataset successfully split and formatted for YOLOv8!")

if __name__ == '__main__':
    setup_directories()
    create_yaml()
    split_data(split_ratio=0.8)