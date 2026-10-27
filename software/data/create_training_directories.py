from pathlib import Path
import os

def create_training_directories(base_path):
    # Define the list of directories to create (定义要创建的目录列表)
    directories = [
        "labels",
        "labels/train_original",
        "labels/val_original",
        "images",
        "images/val",
        "images/train"
    ]
    
    # Traverse the directory list and create each directory (遍历目录列表并创建每个目录)
    for directory in directories:
        path = os.path.join(base_path, directory)
        os.makedirs(path, exist_ok=True)
        print(f"创建目录: {path}")

if __name__ == "__main__":
    base_path = str(Path(__file__).resolve().parents[2] / "software/data")  # Base path (基础路径)
    create_training_directories(base_path)
