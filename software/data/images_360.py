from pathlib import Path
import cv2
import os
import shutil
import xml.etree.ElementTree as ET

def rotate_image(image, angle, cx, cy):
    """
    Rotate the image by a specific angle around a given center point (cx, cy).
    """
    (h, w) = image.shape[:2]
    center = (cx, cy)  # Use the specified center point for rotation (使用给定的中心点进行旋转)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated

def process_images(base_name, cx, cy, image_dir, save_dir):
    # Clear the save path (清空保存路径)
    save_image_dir = os.path.join(save_dir, f'{base_name}_train')
    if os.path.exists(save_image_dir):
        shutil.rmtree(save_image_dir)   # Delete the folder and all its contents (删除文件夹及其中所有文件)
    os.makedirs(save_image_dir, exist_ok=True)

    # Load image (加载图像)
    image_path = os.path.join(image_dir, f'{base_name}.jpg')
    image = cv2.imread(image_path)

    # Rotate the image and save (旋转图像并保存)
    for angle in range(0, 360, 2):
        rotated_image = rotate_image(image, angle, cx, cy)
        save_image_path = os.path.join(save_image_dir, f'{base_name}_{angle}.jpg')
        cv2.imwrite(save_image_path, rotated_image)

    print(f"180张旋转后的图像已生成并保存到 {save_image_dir} 目录。")


def get_cx_cy_from_xml(xml_file_path):
    """
    Get the cx and cy values from the given XML file.(从给定的 XML 文件中获取 cx 和 cy 元素的值。)
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    
    # Find the 'robndbox' element and get the values of 'cx' and 'cy' elements inside it() 查找 'robndbox' 元素，并获取其内部的 'cx' 和 'cy' 元素的值)
    robndbox_element = root.find('.//robndbox')
    if robndbox_element is not None:
        cx_element = robndbox_element.find('cx')
        cy_element = robndbox_element.find('cy')
        
        if cx_element is not None and cy_element is not None:
            cx = float(cx_element.text)
            cy = float(cy_element.text)
            return cx, cy
        else:
            raise ValueError(f"'cx' 或 'cy' 元素在文件 {xml_file_path} 中未找到")
    else:
        raise ValueError(f"'robndbox' 元素在文件 {xml_file_path} 中未找到")

# List of names to be processed (需要处理的名称列表)
# Only modify these names (只需要修改这些名称)
names = ['Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate', 'StorageBattery', 'Toothbrush']


# Basic path settings (基本路径设置)
xml_input_dir = str(Path(__file__).resolve().parents[2] / 'software/data/JPEGImages/zrol_image')
save_dir = str(Path(__file__).resolve().parents[2] / 'software/data/JPEGImages')

# Traverse the name list and process each name (遍历名称列表并处理每个名称)
for base_name in names:
    # Get cx and cy values from the Marker.xml file of the current name (获取当前名称的 Marker.xml 文件中的 cx 和 cy 值)
    xml_file_path = os.path.join(xml_input_dir, f'{base_name}.xml')
    cx, cy = get_cx_cy_from_xml(xml_file_path)
    
    # Process the image and save (处理图像并保存)
    process_images(base_name, cx, cy, xml_input_dir, save_dir)
