from pathlib import Path
import sys
 
sys.path.append(str(Path(__file__).resolve().parents[2] / 'software/ultralytics/') + '/')
 
from ultralytics.data.converter import convert_dota_to_yolo_obb
convert_dota_to_yolo_obb(str(Path(__file__).resolve().parents[2] / 'software/data'))
