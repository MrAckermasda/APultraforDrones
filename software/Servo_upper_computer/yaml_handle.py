from pathlib import Path
import yaml

lab_file_path = str(Path(__file__).resolve().parents[2] / 'src/app/config/lab_config.yaml')
servo_file_path = str(Path(__file__).resolve().parents[2] / 'software/Servo_upper_computer/servo_config.yaml')
def get_yaml_data(yaml_file):
    file = open(yaml_file, 'r', encoding='utf-8')
    file_data = file.read()
    file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

def save_yaml_data(data, yaml_file):
    file = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, file)
    file.close()
