from pathlib import Path
from ultralytics import YOLO
# Load a model
model = YOLO(str(Path(__file__).resolve().parents[2] / 'software/data/yolov8n-obb.pt'))  
# Train the model
results = model.train(data=str(Path(__file__).resolve().parents[2] / 'software/data/data.yaml'), epochs=50, batch=10, imgsz=640,amp=False)
