import os
from ultralytics import YOLO

def main():
    # 1. Define the path to your data.yaml file
    # This file should be provided by your FSOCO dataset download (e.g., from Roboflow or Kaggle)
    # It tells YOLO where your train/val images and labels are located.
    data_yaml_path = 'dataset/data_split/config.yaml' 
    
    if not os.path.exists(data_yaml_path):
        print(f"Error: Could not find '{data_yaml_path}'.")
        print("Please download the FSOCO YOLO dataset and ensure the 'data.yaml' is in the correct path.")
        return

    # 2. Load a pre-trained base model (yolov8n.pt is the nano model, good for real-time inference)
    print("Loading YOLOv8 nano model...")
    model = YOLO('yolov8n.pt')

    # 3. Train the model
    # Adjust epochs and batch size based on your GPU capabilities
    print("Starting training on FSOCO dataset...")
    results = model.train(
        data=data_yaml_path,
        epochs=100,           # Number of training epochs
        imgsz=640,            # Image size
        batch=16,             # Batch size
        name='fsoco_model',   # Name of the output directory
        device='0',           # Use '0' for GPU, or 'cpu' if no GPU is available
        workers=8             # Number of dataloader workers
    )
    
    print("Training complete!")
    print("Your best model is saved at: runs/detect/fsoco_model/weights/best.pt")

if __name__ == '__main__':
    main()
