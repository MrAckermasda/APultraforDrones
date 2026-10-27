#!/usr/bin/env python3
# coding: utf-8
# Robotic arm hand-eye calibration program, generating hand2cam_tf_matrix(机械臂手眼标定程序，生成 hand2cam_tf_matrix)
from pathlib import Path
import sys
import os
import yaml
import math
import rclpy
import numpy as np
include_dir = str(Path(__file__).resolve().parents[2] / "software/") + '/'
sys.path.append(include_dir)
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, Empty
from sensor_msgs.msg import Image, CameraInfo
from dt_apriltags import Detector
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller_msgs.msg import ServosPosition
from include import message_main, camera_thread
from ui import Ui_Form
from handeye_calibration import get_cam2hand
import cv2

class HandEyeCalibration(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.config_path = str(Path(__file__).resolve().parents[2] / "src/app/config/") + '/'
        self.config_file = "transform.yaml"
        self.tag_size = 0.025   # AprilTag tag size (AprilTag 标签尺寸)
        self.intrinsic = None
        self.distortion = None
        self.enter = False
        self.bridge = CvBridge()

        # Initialize AprilTag detector (初始化 AprilTag 检测器)
        self.at_detector = Detector(
            searchpath=['apriltags'],
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ROS2 topics and services (ROS2 话题和服务)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.get_current_pose_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        # Camera thread (相机线程)
        self.camera = camera_thread.ROS_Camera(self)
        self.camera.raw_data.connect(self.image_callback)

        # Qt interface (Qt 界面)
        self.app = QApplication([])
        self.window = CalibrationWindow(self)
        self.window.show()

        # Initialize thred (初始化线程)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=ReentrantCallbackGroup())

    def init_process(self):
        self.timer.cancel()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Hand-eye calibration node started')

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Entering hand-eye calibration')
        self.enter = True
        self.camera.enter()
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Exiting hand-eye calibration')
        self.enter = False
        self.camera.exit()
        response.success = True
        response.message = "exit"
        return response

    def image_callback(self, cv_image):
        if not self.enter:
            return
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.intrinsic is not None and self.distortion is not None:
            tags = self.at_detector.detect(gray_image, True, (self.intrinsic[0, 0], self.intrinsic[1, 1], self.intrinsic[0, 2], self.intrinsic[1, 2]), self.tag_size)
            if tags:
                for tag in tags:
                    if tag.tag_id == 1: # Assume using AprilTag with ID=1 (假设使用 ID=1 的 AprilTag)
                        pose_t = tag.pose_t.flatten()
                        pose_r = tag.pose_R
                        roll, pitch, yaw = self.mat_to_xyz_euler(np.eye(4), degrees=True)[1]
                        self.window.xyz_rpy = [pose_t[0], pose_t[1], pose_t[2], roll, pitch, yaw]
                        # Update UI display (更新 UI 显示)
                        self.window.label_x.setText(f"{pose_t[0]:.4f}")
                        self.window.label_y.setText(f"{pose_t[1]:.4f}")
                        self.window.label_z.setText(f"{pose_t[2]:.4f}")
                        self.window.label_roll.setText(f"{roll:.4f}")
                        self.window.label_pitch.setText(f"{pitch:.4f}")
                        self.window.label_yaw.setText(f"{yaw:.4f}")
                        break
            else:
                self.window.xyz_rpy = []

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request())
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        roll, pitch, yaw = self.quat_to_euler([pose_r.w, pose_r.x, pose_r.y, pose_r.z], degrees=True)
        return [pose_t.x, pose_t.y, pose_t.z, roll, pitch, yaw]

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def save_yaml_data(self, data, yaml_file):
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f)
        self.set_parameters([rclpy.parameter.Parameter('hand2cam_tf_matrix', rclpy.Parameter.Type.DOUBLE_ARRAY, data['hand2cam_tf_matrix'])])

    # Borrowed from the common module of TagStackup (从 TagStackup 的 common 模块借用)
    def mat_to_xyz_euler(self, mat, degrees=False):
        translation = mat[:3, 3]
        euler = np.array([math.atan2(mat[1, 0], mat[0, 0]),
                          math.atan2(-mat[2, 0], np.sqrt(mat[2, 1]**2 + mat[2, 2]**2)),
                          math.atan2(mat[2, 1], mat[2, 2])])
        if degrees:
            euler = np.degrees(euler)
        return translation, euler

    def quat_to_euler(self, quat, degrees=False):
        w, x, y, z = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        if degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)
        return roll, pitch, yaw

class CalibrationWindow(QWidget, Ui_Form):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setupUi(self)
        self.setWindowTitle("Hand-Eye Calibration")
        self.xyz_rpy = []
        self.message = message_main.Message(self)
        self.init_ui()

    def init_ui(self):
        # Connect button signals (连接按钮信号)
        self.pushButton_add.clicked.connect(self.add_data)
        self.pushButton_calibrate.clicked.connect(self.calibrate)
        self.pushButton_save.clicked.connect(self.save_data)
        self.pushButton_delete.clicked.connect(self.delete_data)
        self.pushButton_open.clicked.connect(self.load_data)
        self.pushButton_update.clicked.connect(self.update_data)

        # Initialize table (初始化表格)
        self.tableWidget.setHorizontalHeaderLabels(['ID', 'Type', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)

    def add_one_line(self, row, col, item):
        item_ = QTableWidgetItem(str(item))
        item_.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.tableWidget.setItem(row, col, item_)

    def add_data(self):
        if not self.xyz_rpy:
            self.message.tips('No AprilTag detected!')
            return

        # Get robot pose (获取手臂位姿)
        hand_pose = self.node.get_endpoint()
        # Get camera pose (获取相机位姿)
        cam_pose = self.xyz_rpy

        # Add to table (添加到表格)
        row_count = self.tableWidget.rowCount()
        self.tableWidget.insertRow(row_count)
        self.add_one_line(row_count, 0, str(int(row_count / 2) + 1))
        self.add_one_line(row_count, 1, 'hand')
        for i, value in enumerate(hand_pose):
            self.add_one_line(row_count, i + 2, round(value, 4))

        self.tableWidget.insertRow(row_count + 1)
        self.add_one_line(row_count + 1, 0, '')
        self.add_one_line(row_count + 1, 1, 'cam')
        for i, value in enumerate(cam_pose):
            self.add_one_line(row_count + 1, i + 2, round(value, 4))

        self.tableWidget.setSpan(row_count, 0, 2, 1)

    def update_data(self):
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            self.message.tips('Please select a row to update!')
            return
        if not self.xyz_rpy:
            self.message.tips('No AprilTag detected!')
            return

        row = selected_rows[0].row()
        if row % 2 != 0:
            row -= 1 # Ensure selecting the 'hand' row (确保选择的是 hand 行)

        # Get robot pose (获取手臂位姿)
        hand_pose = self.node.get_endpoint()
        # Get camera pose (获取相机位姿)
        cam_pose = self.xyz_rpy

        # Update table (更新表格)
        for i, value in enumerate(hand_pose):
            self.add_one_line(row, i + 2, round(value, 4))
        for i, value in enumerate(cam_pose):
            self.add_one_line(row + 1, i + 2, round(value, 4))

    def delete_data(self):
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            self.message.tips('Please select a row to delete!')
            return
        row = selected_rows[0].row()
        if row % 2 != 0:
            row -= 1
        self.tableWidget.removeRow(row + 1)
        self.tableWidget.removeRow(row)
        # Update ID(更新 ID)
        for i in range(0, self.tableWidget.rowCount(), 2):
            self.add_one_line(i, 0, str(int(i / 2) + 1))
            self.add_one_line(i + 1, 0, '')
            self.tableWidget.setSpan(i, 0, 2, 1)

    def load_data(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.ExistingFile)
        file_dialog.setNameFilter('npz Files(*.npz)')
        file_path, _ = file_dialog.getOpenFileName(self, 'Open File', self.node.config_path, 'npz Files(*.npz)')
        if file_path:
            if not file_path.endswith('.npz'):
                self.message.tips('File format is not .npz!')
                return
            data = np.load(file_path)
            hand_data = data['hand']
            cam_data = data['cam']
            self.tableWidget.setRowCount(0)
            self.tableWidget.clearContents()
            for i in range(len(hand_data)):
                row_count = self.tableWidget.rowCount()
                self.tableWidget.insertRow(row_count)
                self.add_one_line(row_count, 0, str(i + 1))
                self.add_one_line(row_count, 1, 'hand')
                for j, value in enumerate(hand_data[i]):
                    self.add_one_line(row_count, j + 2, round(value, 4))

                self.tableWidget.insertRow(row_count + 1)
                self.add_one_line(row_count + 1, 0, '')
                self.add_one_line(row_count + 1, 1, 'cam')
                for j, value in enumerate(cam_data[i]):
                    self.add_one_line(row_count + 1, j + 2, round(value, 4))

                self.tableWidget.setSpan(row_count, 0, 2, 1)

    def get_all_data(self):
        hand_data = []
        cam_data = []
        row_count = self.tableWidget.rowCount()
        for i in range(0, row_count, 2):
            hand_data_temp = []
            cam_data_temp = []
            for j in range(2, 8):
                hand = float(self.tableWidget.item(i, j).text())
                cam = float(self.tableWidget.item(i + 1, j).text())
                hand_data_temp.append(hand)
                cam_data_temp.append(cam)
            hand_data.append(hand_data_temp)
            cam_data.append(cam_data_temp)
        return hand_data, cam_data

    def calibrate(self):
        hand_data, cam_data = self.get_all_data()
        if not hand_data or not cam_data:
            self.message.tips('No data available for calibration!')
            return
        try:
            X = get_cam2hand(hand_data, cam_data)
            self.node.hand2cam_tf_matrix = X.tolist()
            self.message.tips('Calibration completed!')
        except Exception as e:
            self.message.tips(f'Calibration failed: {str(e)}')

    def save_data(self):
        if not hasattr(self.node, 'hand2cam_tf_matrix'):
            self.message.tips('No calibration data to save!')
            return
        config = {'hand2cam_tf_matrix': self.node.hand2cam_tf_matrix}
        self.node.save_yaml_data(config, os.path.join(self.node.config_path, self.node.config_file))
        self.message.tips('Calibration data saved to transform.yaml')

def main():
    rclpy.init()
    node = HandEyeCalibration('hand_eye_calibration')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Start ROS2 thread (启动 ROS2 线程)
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    # Run Qt event loop (运行 Qt 事件循环)
    node.window.app.exec_()
    # Cleanup (清理)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
