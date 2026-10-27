from pathlib import Path
from ultralytics import YOLO
# Load a model
model = YOLO('yolo11s-obb.pt')  
# Train the model
results = model.train(data=str(Path(__file__).resolve().parents[2] / 'software/collect_picture/my_data/data.yaml'),epochs=30, batch=4, imgsz=320,amp=False)




