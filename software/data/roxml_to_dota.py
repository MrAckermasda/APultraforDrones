#File name (文件名称)   ：roxml_to_dota.py
#Function description(功能描述)  ：Convert rolabelimg's xml file to dota's recognizable xml file(把rolabelimg标注的xml文件转换成dota能识别的xml文件，)
#                                 then convert it to dota format txt file. (再转换成dota格式的txt文件)
#                                 Convert bounding box cx,cy,w,h,angle or rectangle box cx,cy,w,h to four-point coordinates x1,y1,x2,y2,x3,y3,x4,y4
#                                 (把旋转框 cx,cy,w,h,angle，或者矩形框cx,cy,w,h,转换成四点坐标x1,y1,x2,y2,x3,y3,x4,y4)


from pathlib import Path
import os
import xml.etree.ElementTree as ET
import math

# Shared label list (共享的标签列表)
cls_list = ['no-mask', 'mask']

def edit_xml(xml_file, dotaxml_file):
    """
    修改xml文件
    :param xml_file:xml文件的路径
    :param dotaxml_file: 生成的dota格式xml文件的路径
    :return: None
    """

 
    tree = ET.parse(xml_file)
    objs = tree.findall('object')
    for ix, obj in enumerate(objs):
        x0 = ET.Element("x0")   # Create node (创建节点)
        y0 = ET.Element("y0")
        x1 = ET.Element("x1")
        y1 = ET.Element("y1")
        x2 = ET.Element("x2")
        y2 = ET.Element("y2")
        x3 = ET.Element("x3")
        y3 = ET.Element("y3")

 
        if (obj.find('robndbox') == None):
            obj_bnd = obj.find('bndbox')
            obj_xmin = obj_bnd.find('xmin')
            obj_ymin = obj_bnd.find('ymin')
            obj_xmax = obj_bnd.find('xmax')
            obj_ymax = obj_bnd.find('ymax')
            # To prevent negative coordinate values(以防有负值坐标)
            xmin = max(float(obj_xmin.text), 0)
            ymin = max(float(obj_ymin.text), 0)
            xmax = max(float(obj_xmax.text), 0)
            ymax = max(float(obj_ymax.text), 0)
            obj_bnd.remove(obj_xmin)  # Remove node (删除节点)
            obj_bnd.remove(obj_ymin)
            obj_bnd.remove(obj_xmax)
            obj_bnd.remove(obj_ymax)
            x0.text = str(xmin)
            y0.text = str(ymax)
            x1.text = str(xmax)
            y1.text = str(ymax)
            x2.text = str(xmax)
            y2.text = str(ymin)
            x3.text = str(xmin)
            y3.text = str(ymin)
        else:
            obj_bnd = obj.find('robndbox')
            obj_bnd.tag = 'bndbox'   # Modify node name (修改节点名)
            obj_cx = obj_bnd.find('cx')
            obj_cy = obj_bnd.find('cy')
            obj_w = obj_bnd.find('w')
            obj_h = obj_bnd.find('h')
            obj_angle = obj_bnd.find('angle')
            cx = float(obj_cx.text)
            cy = float(obj_cy.text)
            w = float(obj_w.text)
            h = float(obj_h.text)
            angle = float(obj_angle.text)
            obj_bnd.remove(obj_cx)   # Remove node (删除节点)
            obj_bnd.remove(obj_cy)
            obj_bnd.remove(obj_w)
            obj_bnd.remove(obj_h)
            obj_bnd.remove(obj_angle)
 
            x0.text, y0.text = rotatePoint(cx, cy, cx - w / 2, cy - h / 2, -angle)
            x1.text, y1.text = rotatePoint(cx, cy, cx + w / 2, cy - h / 2, -angle)
            x2.text, y2.text = rotatePoint(cx, cy, cx + w / 2, cy + h / 2, -angle)
            x3.text, y3.text = rotatePoint(cx, cy, cx - w / 2, cy + h / 2, -angle)
 
        # obj.remove(obj_type)  # Remove node (删除节点)
        obj_bnd.append(x0)  # Add node (新增节点)
        obj_bnd.append(y0)
        obj_bnd.append(x1)
        obj_bnd.append(y1)
        obj_bnd.append(x2)
        obj_bnd.append(y2)
        obj_bnd.append(x3)
        obj_bnd.append(y3)
 
        tree.write(dotaxml_file, method='xml', encoding='utf-8')  # Update xml file (更新xml文件)
 
 
# Convert to four-point coordinates (转换成四点坐标)
def rotatePoint(xc, yc, xp, yp, theta):
    """
    将中心点(cx, cy) 和 点 (xp, yp) 绕 (xc, yc) 旋转 theta 角度
    :return: 返回新的点坐标 (x, y)
    """
    xoff = xp - xc
    yoff = yp - yc
    cosTheta = math.cos(theta)
    sinTheta = math.sin(theta)
    pResx = cosTheta * xoff + sinTheta * yoff
    pResy = - sinTheta * xoff + cosTheta * yoff
    return str(int(xc + pResx)), str(int(yc + pResy))
 
 
def totxt(xml_path, out_path):
    # Path where you want to save the generated txt files, can be modified here (想要生成的txt文件保存的路径，这里可以自己修改)
 
    files = os.listdir(xml_path)
    i = 0
    for file in files:
 
        tree = ET.parse(xml_path + os.sep + file)
        root = tree.getroot()
 
        name = file.split('.')[0]
 
        output = out_path + '/' + name + '.txt'
        file = open(output, 'w')
        i = i + 1
        objs = tree.findall('object')
        for obj in objs:
            cls = obj.find('name').text
            box = obj.find('bndbox')
            x0 = int(float(box.find('x0').text))
            y0 = int(float(box.find('y0').text))
            x1 = int(float(box.find('x1').text))
            y1 = int(float(box.find('y1').text))
            x2 = int(float(box.find('x2').text))
            y2 = int(float(box.find('y2').text))
            x3 = int(float(box.find('x3').text))
            y3 = int(float(box.find('y3').text))
            if x0 < 0:
                x0 = 0
            if x1 < 0:
                x1 = 0
            if x2 < 0:
                x2 = 0
            if x3 < 0:
                x3 = 0
            if y0 < 0:
                y0 = 0
            if y1 < 0:
                y1 = 0
            if y2 < 0:
                y2 = 0
            if y3 < 0:
                y3 = 0
            for cls_index, cls_name in enumerate(cls_list):
                if cls == cls_name:
                    file.write("{} {} {} {} {} {} {} {} {} {}\n".format(x0, y0, x1, y1, x2, y2, x3, y3, cls, cls_index))
        file.close()
        # print(output)
        print(i)
 
 
if __name__ == '__main__':
    # Label name list:Added image is for supplementary images (标签类名列表  增加的image为补充图片)
    names = ['no-mask','mask']

    # Traverse label name list and process each label name (遍历类名列表并处理每个类名)
    for base_name in names:
        # -----**** Step 1: Convert all xml files to xml files with rotation bounding boxes ****   Modify to your own path----- (-----**** 第一步：把xml文件统一转换成旋转框的xml文件 ****    修改为自己的路径-----)
        roxml_path = str(Path(__file__).resolve().parents[2] / f'software/data/Annotations')   # Folder of annotated xml files (标注好的xml文件夹)
        dotaxml_path = str(Path(__file__).resolve().parents[2] / f'software/data/val_dota')    # New folder to store (新建文件夹储存)
        out_path = str(Path(__file__).resolve().parents[2] / f'software/data/val_original')    # New folder to store (新建文件夹储存)

        # Create directory if directory does not exist (创建目录，如果目录不存在)
        os.makedirs(dotaxml_path, exist_ok=True)
        os.makedirs(out_path, exist_ok=True)

        filelist = os.listdir(roxml_path)
        for file in filelist:
            edit_xml(os.path.join(roxml_path, file), os.path.join(dotaxml_path, file))

        # -----**** Step 2: Convert rotation bounding box xml files to txt format ****----- (-----**** 第二步：把旋转框xml文件转换成txt格式 ****-----)
        totxt(dotaxml_path, out_path)
