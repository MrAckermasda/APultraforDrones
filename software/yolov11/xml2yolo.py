import os
import xml.etree.ElementTree as ET
import argparse
import yaml
import math
import random

def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert XML annotations to YOLO format')
    parser.add_argument('--data', type=str, required=True, help='Path to dataset root directory')
    parser.add_argument('--yaml', type=str, required=True, help='Path to output YAML file')
    return parser.parse_args()

def load_class_mapping(data_dir):
    """从 classes.names 文件加载类别映射"""
    class_names_file = os.path.join(data_dir, 'classes.names')
    
    if not os.path.exists(class_names_file):
        raise FileNotFoundError(f"classes.names file not found at {class_names_file}")
    
    with open(class_names_file, 'r') as f:
        classes = [line.strip() for line in f.readlines() if line.strip()]
    
    class_mapping = {class_name: idx for idx, class_name in enumerate(classes)}
    print(f"Loaded {len(classes)} classes: {classes}")
    return class_mapping, classes

def normalize_coordinates(cx, cy, w, h, angle, img_width, img_height):
    norm_cx = cx / img_width
    norm_cy = cy / img_height
    norm_w = w / img_width
    norm_h = h / img_height
    # 角度从度转换为弧度
    norm_angle = angle * math.pi / 180.0
    return norm_cx, norm_cy, norm_w, norm_h, norm_angle

def convert_xml_to_txt(xml_file, output_dir, class_mapping):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # 获取图片尺寸
    size_elem = root.find('size')
    if size_elem is None:
        print(f"Warning: No size element found in {xml_file}, skipping.")
        return None
    
    img_width = int(size_elem.find('width').text)
    img_height = int(size_elem.find('height').text)

    # 获取文件名
    filename_elem = root.find('filename')
    if filename_elem is None or filename_elem.text is None:
        print(f"Warning: No filename found in {xml_file}, skipping.")
        return None
        
    filename = os.path.splitext(filename_elem.text)[0]
    txt_file_path = os.path.join(output_dir, f"{filename}.txt")

    with open(txt_file_path, 'w') as txt_file:
        for obj in root.findall('object'):
            name_elem = obj.find('name')
            if name_elem is None or name_elem.text is None:
                continue
                
            class_name = name_elem.text
            if class_name not in class_mapping:
                print(f"Warning: class '{class_name}' not in CLASS_MAPPING, skipping.")
                continue

            class_id = class_mapping[class_name]
            robndbox = obj.find('robndbox')
            
            if robndbox is None:
                print(f"Warning: No robndbox found for object in {xml_file}, skipping.")
                continue

            try:
                cx = float(robndbox.find('cx').text)
                cy = float(robndbox.find('cy').text)
                w = float(robndbox.find('w').text)
                h = float(robndbox.find('h').text)
                angle = float(robndbox.find('angle').text)  # 添加角度
            except (AttributeError, ValueError) as e:
                print(f"Warning: Error parsing coordinates in {xml_file}: {e}")
                continue

            # 归一化（包括角度）
            norm_cx, norm_cy, norm_w, norm_h, norm_angle = normalize_coordinates(cx, cy, w, h, angle, img_width, img_height)

            # 写入文件，YOLO OBB 格式：class_id cx cy width height angle
            txt_file.write(f"{class_id} {norm_cx:.6f} {norm_cy:.6f} {norm_w:.6f} {norm_h:.6f} {norm_angle:.6f}\n")

    print(f"Converted {xml_file} to {txt_file_path}")
    return filename  # 返回转换成功的文件名

def convert_all_xml_in_dir(xml_dir, output_dir, class_mapping):
    os.makedirs(output_dir, exist_ok=True)
    converted_count = 0
    converted_files = []  # 保存转换成功的文件名列表
    
    for file_name in os.listdir(xml_dir):
        if file_name.endswith('.xml'):
            xml_file_path = os.path.join(xml_dir, file_name)
            filename = convert_xml_to_txt(xml_file_path, output_dir, class_mapping)
            if filename is not None:
                converted_count += 1
                converted_files.append(filename)
    
    print(f"Conversion completed. Total files converted: {converted_count}")
    return converted_files  # 返回转换成功的文件名列表

def create_train_val_split(data_dir, image_files, split_ratio=0.8, seed=42):
    """创建训练集和验证集的文件列表"""
    # 确保随机分割可重现
    random.seed(seed)
    
    # 随机打乱文件列表
    random.shuffle(image_files)
    
    # 计算分割点
    split_index = int(len(image_files) * split_ratio)
    train_files = image_files[:split_index]
    val_files = image_files[split_index:]
    
    # 创建ImageSets目录
    imagesets_dir = os.path.join(data_dir, 'ImageSets')
    os.makedirs(imagesets_dir, exist_ok=True)
    
    # 获取数据集的绝对路径
    data_dir_abs = os.path.abspath(data_dir)
    
    # 写入训练集文件 - 使用绝对路径
    train_txt = os.path.join(imagesets_dir, 'train.txt')
    with open(train_txt, 'w') as f:
        for filename in train_files:
            # 使用绝对路径
            image_abs_path = os.path.join(data_dir_abs, 'JPEGImages', filename + '.jpg')
            f.write(f"{image_abs_path}\n")
    
    # 写入验证集文件 - 使用绝对路径
    val_txt = os.path.join(imagesets_dir, 'val.txt')
    with open(val_txt, 'w') as f:
        for filename in val_files:
            image_abs_path = os.path.join(data_dir_abs, 'JPEGImages', filename + '.jpg')
            f.write(f"{image_abs_path}\n")
    
    print(f"Created train/val split: {len(train_files)} training, {len(val_files)} validation")
    print(f"Train file: {train_txt}")
    print(f"Val file: {val_txt}")
    
    return train_txt, val_txt

def create_yaml_file(yaml_path, classes, data_dir, image_files=None):
    """创建 YAML 配置文件"""
    train_txt = os.path.join(data_dir, 'ImageSets', 'train.txt')
    val_txt = os.path.join(data_dir, 'ImageSets', 'val.txt')
    
    # 如果train.txt和val.txt不存在，且提供了图像文件列表，则自动创建
    if (not os.path.exists(train_txt) or not os.path.exists(val_txt)) and image_files:
        print("Automatically creating train/val split...")
        train_txt, val_txt = create_train_val_split(data_dir, image_files)
    else:
        if not os.path.exists(train_txt):
            print(f"Warning: Training file {train_txt} not found")
        if not os.path.exists(val_txt):
            print(f"Warning: Validation file {val_txt} not found")
    
    # 获取数据集的绝对路径
    data_dir_abs = os.path.abspath(data_dir)
    
    yaml_data = {
        'path': data_dir_abs,  # 数据集根目录的绝对路径
        'train': train_txt,    # 训练集文件列表
        'val': val_txt,        # 验证集文件列表
        'names': {i: name for i, name in enumerate(classes)},  # 类别名称映射
        'nc': len(classes)     # 类别数量
    }
    
    with open(yaml_path, 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
    
    print(f"Created YAML configuration file at: {yaml_path}")

def main():
    args = parse_arguments()
    data_dir = args.data
    
    # 加载类别映射
    class_mapping, classes = load_class_mapping(data_dir)
    
    # 设置输入输出路径
    input_xml_dir = os.path.join(data_dir, "Annotations")
    output_txt_dir = os.path.join(data_dir, "JPEGImages")
    
    # 检查输入目录是否存在
    if not os.path.exists(input_xml_dir):
        print(f"Error: Annotations directory not found at {input_xml_dir}")
        return
    
    # 转换 XML 文件，并获取转换成功的文件名列表
    converted_files = convert_all_xml_in_dir(input_xml_dir, output_txt_dir, class_mapping)
    
    # 检查是否有对应的图像文件
    jpeg_dir = os.path.join(data_dir, "JPEGImages")
    valid_image_files = []
    
    for filename in converted_files:
        # 检查多种常见的图像格式
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
        image_found = False
        
        for ext in image_extensions:
            image_path = os.path.join(jpeg_dir, filename + ext)
            if os.path.exists(image_path):
                valid_image_files.append(filename)
                image_found = True
                break
        
        if not image_found:
            print(f"Warning: No corresponding image file found for {filename}")
    
    if not valid_image_files:
        print("Error: No valid image files found for the converted annotations.")
        return
    
    # 创建 YAML 文件，自动生成训练/验证集分割
    create_yaml_file(args.yaml, classes, data_dir, valid_image_files)
    
    print("All operations completed successfully!")

if __name__ == "__main__":
    main()
