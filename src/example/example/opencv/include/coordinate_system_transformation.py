#!/usr/bin/env python3
# encoding: utf-8
# Object pose calculation (物体位姿计算)
from sdk.paths import workspace_path
import os
import cv2
import yaml
import time
import rclpy
import numpy as np
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage, CameraInfo

class CoordinateTransformation(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.bridge = CvBridge()  # Bridge for converting between ROS Image and OpenCV images (ROS Image和OpenCV图像转换桥梁)
        self.K = None  # Camera intrinsic matrix (相机内参矩阵)
        self.white_area_center = None  # Center of white area in world coordinate system (白色区域中心点（世界坐标系）)
        self.camera_type = os.environ.get('CAMERA_TYPE', '')   # Read CAMERA_TYPE environment variable, empty string if not found (读取CAMERA_TYPE环境变量，若无则空字符串)
        self.config_file = 'transform.yaml'  # Config file name for pose calculation (位姿计算配置文件名)
        self.calibration_file = 'calibration.yaml'  # Calibration file name (校准文件名)
        self.config_path = workspace_path("src/app/config/")  # Directory of config files (配置文件目录)

        # Subscribe to image result topic and camera info topic (订阅图像结果话题和相机内参话题)
        self.image_sub = self.create_subscription(RosImage, '/color_detection/result_image', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)

    def get_yaml(self):
        """读取配置文件，获取相机外参和白色区域世界坐标"""
        with open(os.path.join(self.config_path, self.config_file), 'r') as f:
            config = yaml.safe_load(f)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)  # Corners, currently not used (角点，目前没有用到)
            self.white_area_center = np.array(config['white_area_pose_world'])

        tvec = extristric[:1]  # First row is translation vector (第一行平移向量)
        rmat = extristric[1:]  # Remaining rows are rotation matrix (剩余三行旋转矩阵)

        # Call extrinsics translation shift calculation function, requires np.matrix type（调用外参平移偏移计算函数，要求传入np.matrix类型）
        tvec, rmat = common.extristric_plane_shift(
            np.matrix(tvec).reshape((3, 1)),
            np.matrix(rmat),
            0.03
        )

        self.extristric = tvec, rmat

    def camera_info_callback(self, msg):
        """Triggered when camera intrinsic message is received, convert to 3x3 matrix (收到相机内参消息时触发，转换成3x3矩阵)"""
        # Use np.matrix to ensure no .I error when calling common functions（这里用np.matrix，确保调用common时不会报.I错）
        self.K = np.matrix(msg.k).reshape(3, 3)

    def image_callback(self, result_image):
        """Triggered when image message is received, identify target, calculate actual position (收到图像消息时触发，识别目标，计算实际位置)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(result_image, "mono8")  # Convert to grayscale image (转为灰度图)

            if cv_image is not None:
                contours = cv2.findContours(cv_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

                if contours:
                    # Find the largest contour (找最大轮廓)
                    c = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(c)
                    corners = np.int0(cv2.boxPoints(rect))
                    x, y = rect[0][0], rect[0][1]

                    if self.camera_type == 'usb_cam':
                        # Special correction logic for usb_cam can be filled here, left empty to avoid errors (这里可填usb_cam专用校正逻辑，先留空避免报错)
                        pass

                    # Read extrinsics configuration (读取外参配置)
                    self.get_yaml()

                    # Assemble projection matrix, remember to convert to np.matrix（组装投影矩阵，记得转成np.matrix）
                    projection_matrix = np.row_stack((
                        np.column_stack((self.extristric[1], self.extristric[0])),
                        np.array([[0, 0, 0, 1]])
                    ))
                    projection_matrix = np.matrix(projection_matrix)

                    # Convert pixel coordinates to world coordinate system, pass np.matrix type parameters（将像素坐标转换到世界坐标系，传入np.matrix类型参数）
                    world_pose = common.pixels_to_world([[x, y]], self.K, projection_matrix)[0]

                    # Coordinate system adjustment (做坐标系调整)
                    world_pose[0] = -world_pose[0]
                    world_pose[1] = -world_pose[1]

                    # Add white area center coordinates to calculate real coordinates (加上白区中心坐标，计算真实坐标)
                    position = self.white_area_center[:3, 3] + world_pose
                    position[2] = 0.03  # Fixed height (固定高度)

                    # Read calibration parameters for scaling and offset (读取标定参数做缩放和位移)
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['pixel']['offset'])
                    scale = tuple(config_data['pixel']['scale'])
                    for i in range(3):
                        position[i] = position[i] * scale[i] + offset[i]

                    # Print pixel and actual coordinates (打印像素及实际坐标)
                    self.get_logger().info(f"像素坐标为: x: {x:.2f}, y: {y:.2f}")
                    self.get_logger().info(f"实际坐标为： x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
                    time.sleep(1)
                else:
                    self.get_logger().info("未检测到所需识别的颜色，请将色块放置到相机视野内。")

        except Exception as e:
            self.get_logger().error(f"image_callback异常: {e}")

def main():
    node = CoordinateTransformation('coordinate_transformation')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
