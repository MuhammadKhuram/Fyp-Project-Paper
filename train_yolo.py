from ultralytics import YOLO

if __name__ == '__main__':
    print("Starting YOLOv8 Training on Pakistani Weed Dataset...")

    # Load the base YOLOv8 nano model
    model = YOLO('yolov8n.pt')

    # Train the model (doing 10 epochs just to make sure it works first)
    results = model.train(
        data='dataset.yaml',   
        epochs=10,             
        imgsz=640,             
        batch=16,              
        device='cpu'  # IMPORTANT: If you have an NVIDIA GPU, change 'cpu' to 'cuda' to make it 10x faster!
    )

    print("Training Complete! Look inside the 'runs/detect/train' folder for your graphs.")