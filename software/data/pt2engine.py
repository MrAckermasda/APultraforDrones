from pathlib import Path
from ultralytics import YOLO

# Run inference (运行推理)
model = YOLO("best.engine", task="obb")  # Modify to your model path and task type (修改为你的模型路径和任务类型)
results = model(str(Path(__file__).resolve().parents[2] / "software/data/images/train/image_6.jpg"))  # Modify to your image to be detected (修改自己待检测的图片)
