from pathlib import Path
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

def update_xml(root, new_angle, angle_index, base_name):
    # Update file name and path (更新文件名和路径)
    for elem in root.iter('filename'):
        elem.text = f'{base_name}_{angle_index}.jpg'
    for elem in root.iter('path'):
        elem.text = str(Path(__file__).resolve().parents[2] / f'software/data/JPEGImages/{base_name}_train/{base_name}_{angle_index}.jpg')
    
    # Update the angle of the rotated bounding box (更新旋转框的角度)
    for elem in root.iter('angle'):
        # Restrict the new angle to [0, 3.1415926) (将新的角度限制在 [0, 3.1415926) 区间内)
        if new_angle < 0:
            new_angle += 3.1415926
        elif new_angle >= 3.1415926:
            new_angle -= 3.1415926
        elem.text = str(new_angle)
    
    return root

def save_xml(root, filename):
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    with open(filename, "w") as f:
        f.write(xml_str)

def generate_rotated_xml_files(base_name, initial_angle, angle_decrement, xml_input_dir, save_xml_dir, angle_step=2):
    # Create save path (创建保存路径)
    os.makedirs(save_xml_dir, exist_ok=True)

    # According to angle_step, Generate XML files based on angle_step(生成 XML 文件（根据 angle_step）)
    for i in range(0, 360, angle_step):
        current_angle = initial_angle - (i // angle_step) * angle_decrement * angle_step
        # Load the original XML file (加载原始 XML 文件)
        xml_file = os.path.join(xml_input_dir, f'{base_name}.xml')
        tree = ET.parse(xml_file)
        root = tree.getroot()
        
        # Update the content of the XML file (更新 XML 文件内容)
        updated_root = update_xml(root, current_angle, i, base_name)
        
        # Save the new XML file (保存新的 XML 文件)
        save_xml_path = os.path.join(save_xml_dir, f'{base_name}_{i}.xml')
        save_xml(updated_root, save_xml_path)

    print(f"{360 // angle_step} 个 XML 文件已生成到 {save_xml_dir} 并更新角度。")

def get_initial_angle_from_xml(xml_file_path):
    """
    从给定的 XML 文件中获取 angle 元素的值。
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    
    # Assume there is an element named 'angle' in the XML file (假设 XML 文件中存在名为 'angle' 的元素)
    angle_element = root.find('.//robndbox/angle')
    if angle_element is not None:
        return float(angle_element.text)
    else:
        raise ValueError(f"'angle' 元素在文件 {xml_file_path} 中未找到")

# List of class names to be processed (需要处理的类名列表)
names = ['Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate', 'StorageBattery', 'Toothbrush']

# Basic path settings (基本路径设置)
xml_input_dir = str(Path(__file__).resolve().parents[2] / 'software/data/JPEGImages/zrol_image')
save_dir = str(Path(__file__).resolve().parents[2] / 'software/data/JPEGImages')

# Traverse the list of class names and process each class name (遍历类名列表并处理每个类名)
for base_name in names:
    # Get the angle value from the XML file of the current class name (获取当前类名的 XML 文件中的 angle 值)
    xml_file_path = os.path.join(xml_input_dir, f'{base_name}.xml')
    initial_angle = get_initial_angle_from_xml(xml_file_path)
    
    # Generate rotated XML files (生成旋转后的 XML 文件)
    save_xml_dir = os.path.join(save_dir, f'{base_name}_xml')
    generate_rotated_xml_files(base_name, initial_angle, 0.0174533, xml_input_dir, save_xml_dir)
