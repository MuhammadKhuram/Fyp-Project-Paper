import os
from ultralytics import YOLO

if __name__ == '__main__':
    print("🚀 Starting YOLOv8 Local Training on Synthetic Dataset...")
    
    # Load the base YOLOv8 nano model
    model = YOLO('yolov8n.pt')

    # Start the training run
    results = model.train(
        data='data.yaml',      # Points to the yaml file we just generated
        epochs=100,            # 100 epochs as per the paper
        patience=20,           # Auto-stop if it stops learning
        imgsz=640,
        batch=16,              
        plots=True,            # Auto-generate the graphs for your paper!
        
        # === HYPERPARAMETERS & AUGMENTATIONS FROM PAPER ===
        optimizer='AdamW',
        lr0=0.001,
        cos_lr=True,
        weight_decay=0.0005,
        fliplr=0.5,
        flipud=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=10.0,
        translate=0.1,
        mosaic=1.0
    )
    
    print("\n✅ Training Complete!")
    print("Check the 'runs/detect/train' folder for your best.pt weights and paper graphs!")