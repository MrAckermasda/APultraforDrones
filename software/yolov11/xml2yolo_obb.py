import os
import xml.etree.ElementTree as ET
import argparse
import yaml
import math
import numpy as np
import random
from sklearn.model_selection import train_test_split

def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert XML annotations to YOLO OBB format with four corner points')
    parser.add_argument('--data', type=str, required=True, help='Path to dataset root directory')
    parser.add_argument('--yaml', type=str, required=True, help='Path to output YAML file')
    parser.add_argument('--split-ratio', type=float, default=0.8, help='Train/validation split ratio (default: 0.8)')
    parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducible splits (default: 42)')
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

def calculate_corners(cx, cy, w, h, angle_deg):
    """
    计算旋转矩形的四个角点
    :param cx: 中心点x坐标
    :param cy: 中心点y坐标
    :param w: 宽度
    :param h: 高度
    :param angle_deg: 旋转角度（度）
    :return: 四个角点的坐标列表 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    """
    # 将角度转换为弧度
    angle_rad = math.radians(angle_deg)
    
    # 计算旋转矩阵
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # 计算半宽和半高
    half_w = w / 2.0
    half_h = h / 2.0
    
    # 计算未旋转的四个角点（相对于中心点）
    corners = [
        (-half_w, -half_h),  # 左上
        ( half_w, -half_h),  # 右上
        ( half_w,  half_h),  # 右下
        (-half_w,  half_h)   # 左下
    ]
    
    # 旋转并平移角点
    rotated_corners = []
    for x, y in corners:
        # 旋转
        x_rot = x * cos_a - y * sin_a
        y_rot = x * sin_a + y * cos_a
        
        # 平移
        x_final = cx + x_rot
        y_final = cy + y_rot
        
        rotated_corners.append((x_final, y_final))
    
    return rotated_corners

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
                angle = float(robndbox.find('angle').text)
            except (AttributeError, ValueError) as e:
                print(f"Warning: Error parsing coordinates in {xml_file}: {e}")
                continue

            # 计算四个角点
            corners = calculate_corners(cx, cy, w, h, angle)
            
            # 归一化坐标（除以图像宽高）
            normalized_corners = []
            for x, y in corners:
                norm_x = x / img_width
                norm_y = y / img_height
                normalized_corners.extend([norm_x, norm_y])
            
            # 写入文件，YOLO OBB 格式：class_id x1 y1 x2 y2 x3 y3 x4 y4
            corner_str = " ".join(f"{coord:.6f}" for coord in normalized_corners)
            txt_file.write(f"{class_id} {corner_str}\n")

    print(f"Converted {xml_file} to {txt_file_path}")
    return filename  # 返回转换成功的文件名

def convert_all_xml_in_dir(xml_dir, output_dir, class_mapping):
    os.makedirs(output_dir, exist_ok=True)
    converted_count = 0
    converted_files = []  # 保存所有转换成功的文件名
    
    for file_name in os.listdir(xml_dir):
        if file_name.endswith('.xml'):
            xml_file_path = os.path.join(xml_dir, file_name)
            filename = convert_xml_to_txt(xml_file_path, output_dir, class_mapping)
            if filename is not None:
                converted_count += 1
                converted_files.append(filename)
    
    print(f"Conversion completed. Total files converted: {converted_count}")
    return converted_files  # 返回所有转换成功的文件名列表

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

def create_yaml_file(yaml_path, classes, data_dir, split_ratio=0.8, seed=42, image_files=None):
    """创建 YAML 配置文件"""
    train_txt = os.path.join(data_dir, 'ImageSets', 'train.txt')
    val_txt = os.path.join(data_dir, 'ImageSets', 'val.txt')
    
    # 如果train.txt和val.txt不存在，且提供了图像文件列表，则自动创建
    if (not os.path.exists(train_txt) or not os.path.exists(val_txt)) and image_files:
        print("Automatically creating train/val split...")
        train_txt, val_txt = create_train_val_split(data_dir, image_files, split_ratio, seed)
    else:
        if not os.path.exists(train_txt):
            print(f"Warning: Training file {train_txt} not found")
        if not os.path.exists(val_txt):
            print(f"Warning: Validation file {val_txt} not found")
    
    # 获取数据集的绝对路径
    data_dir_abs = os.path.abspath(data_dir)
    
    yaml_data = {
        'path': data_dir_abs,  # 数据集根目录
        'train': train_txt,    # 训练集文件列表
        'val': val_txt,        # 验证集文件列表
        'names': {i: name for i, name in enumerate(classes)},  # 类别名称映射
        'nc': len(classes)     # 类别数量
    }
    
    with open(yaml_path, 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
    
    print(f"Created YAML configuration file at: {yaml_path}")
    return train_txt, val_txt

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
    train_txt, val_txt = create_yaml_file(
        args.yaml, 
        classes, 
        data_dir, 
        args.split_ratio, 
        args.seed, 
        valid_image_files
    )
    
    # 打印分割统计信息
    if os.path.exists(train_txt):
        with open(train_txt, 'r') as f:
            train_count = len(f.readlines())
    
    if os.path.exists(val_txt):
        with open(val_txt, 'r') as f:
            val_count = len(f.readlines())
    
    print(f"Dataset split: {train_count} training images, {val_count} validation images")
    print("All operations completed successfully!")

if __name__ == "__main__":
    main()
