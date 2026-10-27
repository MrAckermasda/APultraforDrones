# #!/usr/bin/env python3
# # encoding: utf:8
# # @data:2023/02/04
# # @author:aiden

from pathlib import Path
import os
import cv2
import sys
import math
import yaml
import numpy as np
import pandas as pd
import threading
from array import array 
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from dt_apriltags import Detector
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from sdk import common
import handeye_calibration
from ui import Ui_Form
import kinematics.transform as transform
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from geometry_msgs.msg import Point, Quaternion

import time 

ROS_NODE_NAME = 'calibration'

class ImageProcessingWorker(QObject):
    finished = pyqtSignal()
    image_processed = pyqtSignal(QImage, list) 

    def __init__(self, at_detector, intrinsic, tag_size, camera_type='default'):
        super().__init__()
        self.at_detector = at_detector
        self.intrinsic = intrinsic
        self.tag_size = tag_size
        self.camera_type = camera_type  
        self.cv_image = None
        self.processing_lock = threading.Lock()

    def process_image(self, cv_image):
        with self.processing_lock:
            self.cv_image = cv_image 

    def run(self):
        while True:
            processed_image = None
            detected_xyz_rpy = []
            with self.processing_lock:
                if self.cv_image is not None:
                    rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
                    gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                    if self.intrinsic is not None:
                        tags = self.at_detector.detect(
                            gray_image, True,
                            (self.intrinsic[0, 0], self.intrinsic[1, 1], self.intrinsic[0, 2], self.intrinsic[1, 2]),
                            self.tag_size
                        )
                        if tags:
                            for tag in tags:
                                if tag.tag_id == 100:
                                    center = (int((tag.corners[0][0] + tag.corners[2][0]) / 2),
                                              int((tag.corners[0][1] + tag.corners[2][1]) / 2))
                                    cv2.circle(rgb_image, center, 5, (0, 255, 0), -1) 
                                    pose_t = tag.pose_t.flatten()
                                    pose_r = tag.pose_R
                                    mat = np.eye(4)
                                    mat[:3, :3] = pose_r
                                    t, euler = common.mat_to_xyz_euler(mat, degrees=True)
                                    roll, pitch, yaw = euler
                                    detected_xyz_rpy = [pose_t[0], pose_t[1], pose_t[2], roll, pitch, yaw]

                    # Resize based on camera_type(根据camera_type决定是否resize)
                    if self.camera_type.lower() == 'aurora':
                        print(f"[ImageProcessingWorker] camera_type={self.camera_type}, resizing image to (640,400)")
                        image_resize = cv2.resize(rgb_image, (640, 400))
                    else:
                        print(f"[ImageProcessingWorker] camera_type={self.camera_type}, using original image size")
                        image_resize = rgb_image

                    h, w, ch = image_resize.shape
                    bytes_per_line = ch * w
                    processed_image = QImage(image_resize.data, w, h, bytes_per_line, QImage.Format_RGB888)
                    self.cv_image = None
            if processed_image is not None:
                self.image_processed.emit(processed_image, detected_xyz_rpy)
            time.sleep(0.01)
            
class CalibrationWindow(Node, QWidget, Ui_Form):
    def __init__(self, name='calibration'):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        QWidget.__init__(self)
        try:
            self.setupUi(self)
            self.set_window_position()
        except Exception as e:
            self.get_logger().error(f"初始化界面失败: {str(e)}")
            raise

        self.config_path = str(Path(__file__).resolve().parents[3] / 'src/peripherals/config/') + '/'
        self.config_file = 'camera_info.yaml'
        self.npz_path = str(Path(__file__).resolve().parents[3] / 'software/hand2cam_tf_matrix_software/calibration/') + '/'

        self.duration = 1.0
        self.xyz_rpy = []
        self.slider_move = True
        self.not_move = False
        self.pulse_dict = {}
        self.tag_size = 0.025
        self.intrinsic = None
        self.distortion = None

        self.hand2cam_tf_matrix = [
            [0.0, 0.0, 1.0, -0.101],
            [-1.0, 0.0, 0.0, 0.01],
            [0.0, -1.0, 0.0, 0.05],
            [0.0, 0.0, 0.0, 1.0]
        ]
        self.get_logger().info(f"初始化 hand2cam_tf_matrix 为默认值: {self.hand2cam_tf_matrix}")

        self.init_pose = {
            'position': [0.13095696510126037, 0.0, 0.0917891053187185],
            'pitch': 0.0,
            'pitch_range': [-180.0, 180.0],
            'resolution': 0.1
        }

        self.pulse_min = 0
        self.pulse_max = 1000

        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableWidget.setSortingEnabled(False)

        self.pushButton_calibrate.clicked.connect(lambda: self.button_clicked('calibrate'))
        self.pushButton_init.clicked.connect(lambda: self.button_clicked('init'))
        self.pushButton_reset.clicked.connect(lambda: self.button_clicked('reset'))
        self.pushButton_open.clicked.connect(lambda: self.table_changed('open'))
        self.pushButton_save.clicked.connect(lambda: self.table_changed('save'))
        self.pushButton_add.clicked.connect(lambda: self.table_changed('add'))
        self.pushButton_update.clicked.connect(lambda: self.table_changed('update'))
        self.pushButton_delete.clicked.connect(lambda: self.table_changed('delete'))
        self.tableWidget.itemSelectionChanged.connect(self.select)

        # Hide deviation buttons as they are no longer used
        self.pushButton_read_dev.hide()
        self.pushButton_save_dev.hide()

        self.servo_num = 6
        self.pulse_value = [500] * self.servo_num

        self.horizontalslider_object = []
        self.label_servo_object = []

        for i in range(self.servo_num):
            slider = self.findChild(QSlider, f'horizontalSlider_servo{i + 1}')
            label = self.findChild(QLabel, f'label_servo{i + 1}')
            if slider is None or label is None:
                self.get_logger().error(f"未找到舵机 {i + 1} 的控件: slider='horizontalSlider_servo{i + 1}', label='label_servo{i + 1}'")
                raise RuntimeError(f"缺少舵机 {i + 1} 的界面控件")
            slider.setMinimum(self.pulse_min)
            slider.setMaximum(self.pulse_max)
            slider.setValue(self.pulse_value[i])
            label.setText(str(self.pulse_value[i]))
            self.horizontalslider_object.append(slider)
            self.label_servo_object.append(label)
            slider.valueChanged.connect(lambda _, idx=i: self.horizontalSlider_move(f'servo{idx + 1}'))

        self.bridge = CvBridge()
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

        self.image_processing_thread = QThread()
        self.image_worker = ImageProcessingWorker(self.at_detector, self.intrinsic, self.tag_size)
        self.image_worker.moveToThread(self.image_processing_thread)
        self.image_processing_thread.started.connect(self.image_worker.run)
        self.image_worker.image_processed.connect(self.update_image_display)
        self.image_processing_thread.start()
        self.get_logger().info("Image processing thread started.")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, qos)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        self.kinematics_fk_client = self.create_client(SetJointValue, '/kinematics/set_joint_value_target')
        self.kinematics_ik_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        
        self.get_logger().info("Waiting for kinematics services...")
        self.kinematics_fk_client.wait_for_service()
        self.kinematics_ik_client.wait_for_service()
        self.get_logger().info("Kinematics services connected.")

        calibration_config_file_path = os.path.join(self.config_path, self.config_file)
        if os.path.exists(calibration_config_file_path):
            try:
                with open(calibration_config_file_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                if 'hand2cam_tf_matrix' in config_data:
                    self.hand2cam_tf_matrix = config_data['hand2cam_tf_matrix']
                    self.get_logger().info(f"从配置文件加载 hand2cam_tf_matrix: {self.hand2cam_tf_matrix}")
                else:
                    self.get_logger().warn("配置文件中未找到 hand2cam_tf_matrix")
            except Exception as e:
                self.get_logger().error(f"从 {calibration_config_file_path} 加载 hand2cam_tf_matrix 失败: {str(e)}")
        else:
            self.get_logger().warn(f"未找到标定配置文件: {calibration_config_file_path}。")

        self.get_logger().info(f'{ROS_NODE_NAME} initialized')
        self.enter_func()

    def set_window_position(self):
        """Center the window on the screen."""
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def camera_info_callback(self, msg):
        """Camera intrinsic parameters callback."""
        try:
            self.intrinsic = np.matrix(msg.k).reshape(3, 3)
            self.distortion = np.array(msg.d)
            self.image_worker.intrinsic = self.intrinsic 
            # self.get_logger().info("Camera intrinsics updated")
        except Exception as e:
            self.get_logger().error(f"Failed to process camera intrinsics: {str(e)}")

    def image_callback(self, ros_image):
        """Image receiving callback. Passes image to worker thread for processing."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.image_worker.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS image for worker: {str(e)}")

    @pyqtSlot(QImage, list)
    def update_image_display(self, qimage_display, xyz_rpy_from_worker):
        """Updates the UI with the processed image and AprilTag data."""
        try:
            self.label_display.setPixmap(QPixmap.fromImage(qimage_display))
            self.xyz_rpy = xyz_rpy_from_worker
        except Exception as e:
            self.get_logger().error(f"Error updating image display: {str(e)}")

    def enter_func(self):
        self.button_clicked('init')

    def closeEvent(self, e):
        result = QMessageBox.question(self, "Close Window Confirmation", "Are you sure you want to quit?",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if result == QMessageBox.Yes:           
            self.image_processing_thread.quit()            
            # self.image_processing_thread.wait()
            
            self.destroy_subscription(self.image_sub)
            self.destroy_subscription(self.camera_info_sub)
            self.destroy_publisher(self.joints_pub)
            
            if hasattr(self, 'kinematics_fk_client') and self.kinematics_fk_client.service_is_ready():
                self.kinematics_fk_client.destroy()
            if hasattr(self, 'kinematics_ik_client') and self.kinematics_ik_client.service_is_ready():
                self.kinematics_ik_client.destroy()
            QWidget.closeEvent(self, e)
        else:
            e.ignore()

    def set_servo_positions_via_publisher(self, pub, duration, pulse_values):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in range(self.servo_num):
            position = ServoPosition()
            position.id = i + 1
            position.position = float(pulse_values[i])
            position_list.append(position)
        msg.position = position_list
        msg.position_unit = "pulse"
        pub.publish(msg)
        self.get_logger().info(f"Setting servo positions: {[(p.id, p.position) for p in position_list]}")

    def set_specified_servo_positions(self, pub, duration, mapped_pulses):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for servo_id, pulse_value in mapped_pulses:
            position = ServoPosition()
            position.id = int(servo_id)
            position.position = float(pulse_value)
            position_list.append(position)
        msg.position = position_list
        msg.position_unit = "pulse"
        pub.publish(msg)
        self.get_logger().info(f"Setting specified servo positions: {[(p.id, p.position) for p in position_list]}")

    def update_pose_display(self, position_data=None, orientation_data=None, rpy_data=None):
        if position_data and orientation_data:
            if not isinstance(position_data, Point) or not isinstance(orientation_data, Quaternion):
                self.get_logger().error(f"Received incorrect types for pose display update: pos={type(position_data)}, ori={type(orientation_data)}")
                return
            self.label_x.setText(str(np.format_float_positional(position_data.x, precision=4)))
            self.label_y.setText(str(np.format_float_positional(position_data.y, precision=4)))
            self.label_z.setText(str(np.format_float_positional(position_data.z, precision=4)))
            orientation_array = [orientation_data.x, orientation_data.y, orientation_data.z, orientation_data.w]
            roll, pitch, yaw = transform.qua2rpy(orientation_array)
            self.label_roll.setText(str(np.format_float_positional(roll, precision=2)))
            self.label_pitch.setText(str(np.format_float_positional(pitch, precision=2)))
            self.label_yaw.setText(str(np.format_float_positional(yaw, precision=2)))
        elif rpy_data is not None and len(rpy_data) == 3:
            self.label_x.setText("N/A")
            self.label_y.setText("N/A")
            self.label_z.setText("N/A")
            roll, pitch, yaw = rpy_data[0], rpy_data[1], rpy_data[2]
            self.label_roll.setText(str(np.format_float_positional(roll, precision=2)))
            self.label_pitch.setText(str(np.format_float_positional(pitch, precision=2)))
            self.label_yaw.setText(str(np.format_float_positional(yaw, precision=2)))
        else:
            self.get_logger().warn("update_pose_display received insufficient or incorrect data.")

    def horizontalSlider_move(self, name):
        """Handle slider movement."""
        servo_index = int(name.replace('servo', '')) - 1
        value = self.horizontalslider_object[servo_index].value()
        value = max(self.pulse_min, min(value, self.pulse_max))
        self.label_servo_object[servo_index].setText(str(value))

        if self.slider_move:
            self.pulse_value[servo_index] = value
            self.set_servo_positions_via_publisher(self.joints_pub, self.duration, self.pulse_value)
            fk_joint_values = [
                float(self.pulse_value[5]),
                float(self.pulse_value[4]),
                float(self.pulse_value[3]),
                float(self.pulse_value[2]),
                float(self.pulse_value[1])
            ]
            request = SetJointValue.Request()
            request.joint_value = fk_joint_values
            future = self.kinematics_fk_client.call_async(request)
            future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        """Callback function to handle Forward Kinematics service response."""
        try:
            response = future.result()
            if response.success and hasattr(response, 'pose') and response.pose:
                position = response.pose.position
                orientation = response.pose.orientation
                self.update_pose_display(position_data=position, orientation_data=orientation)
                self.get_logger().info("Forward Kinematics calculation successful.")
            else:
                self.get_logger().warn(f"Forward Kinematics failed. Success: {response.success}")
                self.label_x.setText("N/A")
                self.label_y.setText("N/A")
                self.label_z.setText("N/A")
                self.label_roll.setText("N/A")
                self.label_pitch.setText("N/A")
                self.label_yaw.setText("N/A")
        except Exception as e:
            self.get_logger().error(f"Forward Kinematics service call failed: {str(e)}")

    def select(self):
        """Handle table row selection."""
        if self.tableWidget.selectedItems():
            row = self.tableWidget.currentRow()
            if not self.not_move:
                current_row = row if row % 2 == 0 else row - 1
                self.slider_move = False
                
                pulses_from_dict = self.pulse_dict[str(int((current_row / 2) + 1))]
                
                for i in range(self.servo_num):
                    self.horizontalslider_object[i].setValue(pulses_from_dict[i])
                    self.label_servo_object[i].setText(str(pulses_from_dict[i]))
                
                self.pulse_value = list(pulses_from_dict)
                self.set_servo_positions_via_publisher(self.joints_pub, self.duration, self.pulse_value)

                fk_joint_values = [
                    float(self.pulse_value[5]),
                    float(self.pulse_value[4]),
                    float(self.pulse_value[3]),
                    float(self.pulse_value[2]),
                    float(self.pulse_value[1])
                ]
                request = SetJointValue.Request()
                request.joint_value = fk_joint_values
                future = self.kinematics_fk_client.call_async(request)
                future.add_done_callback(self.fk_response_callback)
                self.slider_move = True

            self.not_move = True
            if row % 2 == 0:
                for c in range(self.tableWidget.columnCount()):
                    self.tableWidget.item(row, c).setSelected(True)
                    self.tableWidget.item(row + 1, c).setSelected(True)
            else:
                for c in range(self.tableWidget.columnCount()):
                    self.tableWidget.item(row, c).setSelected(True)
                    self.tableWidget.item(row - 1, c).setSelected(True)
            self.not_move = False

    def add_one_line(self, row, col, item):
        item_ = QtWidgets.QTableWidgetItem(str(item))
        item_.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.tableWidget.setItem(row, col, item_)

    def add_line(self, row, item, type_, data):
        self.add_one_line(row, 0, item)
        self.add_one_line(row, 1, type_)
        for i in range(6):
            self.add_one_line(row, i + 2, data[i])

    def save_yaml_data(self, data, yaml_file):
        try:
            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"YAML data saved successfully: {yaml_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML data: {str(e)}")

    def button_clicked(self, name):
        if name == 'calibrate':
            try:
                hand_data, cam_data = self.get_all_data()
                X = handeye_calibration.get_cam2hand(hand_data, cam_data)
                
                self.hand2cam_tf_matrix = X.tolist()
                self.get_logger().info(f"手眼标定结果 (hand2cam_tf_matrix) 已更新为: {self.hand2cam_tf_matrix}")

                config_path = os.path.join(self.config_path, self.config_file)
                config = {}
                if os.path.exists(config_path):
                    with open(config_path, 'r') as f:
                        config = yaml.safe_load(f) or {}

                config['hand2cam_tf_matrix'] = X.tolist()
                self.save_yaml_data(config, config_path)
                QMessageBox.information(self, "Info", "Calibration completed")
            except Exception as e:
                self.get_logger().error(f"Hand-eye calibration failed: {str(e)}")
                QMessageBox.warning(self, "Error", f"Calibration failed: {str(e)}")
        
        elif name == 'reset':
            self.slider_move = False
            try:
                self.pulse_value = [500] * self.servo_num
                for i in range(self.servo_num):
                    self.horizontalslider_object[i].setValue(self.pulse_value[i])
                    self.label_servo_object[i].setText(str(self.pulse_value[i]))

                self.set_servo_positions_via_publisher(self.joints_pub, 2.0, self.pulse_value)
                fk_joint_values = [float(p) for p in [self.pulse_value[5], self.pulse_value[4], self.pulse_value[3], self.pulse_value[2], self.pulse_value[1]]]
                request = SetJointValue.Request(joint_value=fk_joint_values)
                future = self.kinematics_fk_client.call_async(request)
                future.add_done_callback(self.fk_response_callback)
            except Exception as e:
                self.get_logger().error(f"Reset failed: {str(e)}")
                QMessageBox.warning(self, "Error", f"Reset failed: {str(e)}")
            finally:
                self.slider_move = True
        
        elif name == 'init':
            self.slider_move = False
            try:
                request = SetRobotPose.Request()
                request.position = self.init_pose['position']
                request.pitch = self.init_pose['pitch']
                request.pitch_range = self.init_pose['pitch_range']
                request.resolution = self.init_pose['resolution']
                request.duration = self.duration
                future = self.kinematics_ik_client.call_async(request)
                future.add_done_callback(self.ik_response_callback)
            except Exception as e:
                self.get_logger().error(f"Failed to set initial pose: {str(e)}")
                QMessageBox.warning(self, "Error", f"Failed to set initial pose: {str(e)}")
            finally:
                self.slider_move = True

    def ik_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                pulse_values_from_ik = response.pulse
                rpy_from_ik = response.rpy
                
                servos_to_update = []
                if len(pulse_values_from_ik) == 5:
                    servos_to_update.extend([(6, int(pulse_values_from_ik[0])), (5, int(pulse_values_from_ik[1])),
                                             (4, int(pulse_values_from_ik[2])), (3, int(pulse_values_from_ik[3])),
                                             (2, int(pulse_values_from_ik[4]))])
                else:
                    self.get_logger().warn(f"IK returned {len(pulse_values_from_ik)} pulses, expected 5.")

                for servo_id, pulse_value in servos_to_update:
                    self.pulse_value[servo_id - 1] = pulse_value
                    self.horizontalslider_object[servo_id - 1].setValue(pulse_value)
                    self.label_servo_object[servo_id - 1].setText(str(pulse_value))
                
                self.set_specified_servo_positions(self.joints_pub, self.duration, servos_to_update)
                self.update_pose_display(rpy_data=rpy_from_ik)
            else:
                msg = response.message if hasattr(response, 'message') else 'No solution'
                self.get_logger().warn(f"Initial pose IK failed: {msg}")
                QMessageBox.warning(self, "Error", f"Failed to set initial pose: {msg}")
        except Exception as e:
            self.get_logger().error(f"Inverse Kinematics service call failed: {str(e)}")
            QMessageBox.warning(self, "Error", f"Failed to set initial pose: {str(e)}")

    def get_all_data(self):
        hand_data = []
        cam_data = []
        for i in range(0, self.tableWidget.rowCount(), 2):
            hand_data_temp = [float(self.tableWidget.item(i, j).text()) for j in range(2, 8)]
            cam_data_temp = [float(self.tableWidget.item(i + 1, j).text()) for j in range(2, 8)]
            hand_data.append(hand_data_temp)
            cam_data.append(cam_data_temp)
        return hand_data, cam_data

    def table_changed(self, name):
        self.not_move = True
        if name == 'add':
            if not self.xyz_rpy:
                QMessageBox.information(self, "Info", "No AprilTag detected")
            else:
                pose_labels = [self.label_x.text(), self.label_y.text(), self.label_z.text(),
                               self.label_roll.text(), self.label_pitch.text(), self.label_yaw.text()]
                if "N/A" in pose_labels:
                    QMessageBox.warning(self, "Warning", "Current pose is not valid. Cannot add to table.")
                    self.not_move = False
                    return

                RowCont = self.tableWidget.rowCount()
                self.tableWidget.insertRow(RowCont)
                self.add_line(RowCont, str(int((RowCont / 2) + 1)), 'hand', [float(p) for p in pose_labels])
                self.tableWidget.insertRow(RowCont + 1)
                self.add_line(RowCont + 1, '', 'cam', self.xyz_rpy)
                self.tableWidget.setSpan(RowCont, 0, 2, 1)
                self.tableWidget.selectRow(RowCont)
                self.pulse_dict[str(int((RowCont / 2) + 1))] = [int(self.label_servo_object[i].text()) for i in range(self.servo_num)]
        
        elif name == 'delete':
            item = self.tableWidget.currentRow()
            if item == -1:
                QMessageBox.information(self, "Info", "No row selected to delete.")
            else:
                row_to_del = item if item % 2 == 0 else item - 1
                del self.pulse_dict[str(int((row_to_del / 2) + 1))]
                self.tableWidget.removeRow(row_to_del + 1)
                self.tableWidget.removeRow(row_to_del)
        
        elif name == 'update':
            if not self.xyz_rpy:
                QMessageBox.information(self, "Info", "No AprilTag detected")
            else:
                item = self.tableWidget.currentRow()
                if item == -1:
                    QMessageBox.information(self, "Info", "No row selected to update.")
                else:
                    pose_labels = [self.label_x.text(), self.label_y.text(), self.label_z.text(),
                                   self.label_roll.text(), self.label_pitch.text(), self.label_yaw.text()]
                    if "N/A" in pose_labels:
                        QMessageBox.warning(self, "Warning", "Current pose is not valid. Cannot update table.")
                        self.not_move = False
                        return
                    
                    current_row = item if item % 2 == 0 else item - 1
                    self.add_line(current_row, str(int((current_row / 2) + 1)), 'hand', [float(p) for p in pose_labels])
                    self.add_line(current_row + 1, '', 'cam', self.xyz_rpy)
                    self.tableWidget.setSpan(current_row, 0, 2, 1)
                    self.tableWidget.selectRow(current_row)
                    self.pulse_dict[str(int((current_row / 2) + 1))] = [int(self.label_servo_object[i].text()) for i in range(self.servo_num)]
        
        elif name == 'open':
            path, _ = QFileDialog.getOpenFileName(self, 'Open File', self.npz_path, 'npz Files(*.npz)')
            if path:
                data = np.load(path, allow_pickle=True)
                hand_data, cam_data, pulses = data['hand'], data['cam'], data['pulse']
                self.tableWidget.setRowCount(0)
                self.pulse_dict = {}
                for i in range(len(pulses)):
                    self.pulse_dict[str(i + 1)] = list(pulses[i])
                    RowCont = self.tableWidget.rowCount()
                    self.tableWidget.insertRow(RowCont)
                    self.add_line(RowCont, str(i + 1), 'hand', list(hand_data[i]))
                    self.tableWidget.insertRow(RowCont + 1)
                    self.add_line(RowCont + 1, '', 'cam', list(cam_data[i]))
                    self.tableWidget.setSpan(RowCont, 0, 2, 1)
                if self.tableWidget.rowCount() > 0:
                    self.tableWidget.selectRow(0)
        
        elif name == 'save':
            if self.tableWidget.rowCount() == 0:
                QMessageBox.information(self, "Info", "List is empty, no need to save")
            else:
                path, _ = QFileDialog.getSaveFileName(self, 'Save File', self.npz_path, 'npz Files(*.npz)')
                if path:
                    if not path.endswith('.npz'):
                        path += '.npz'
                    hand_data, cam_data = self.get_all_data()
                    pulses_to_save = [self.pulse_dict[key] for key in sorted(self.pulse_dict.keys(), key=int)]
                    np.savez(path, hand=np.array(hand_data), cam=np.array(cam_data), pulse=np.array(pulses_to_save))

        # Re-number all rows after add/delete operations
        new_pulse_dict = {}
        for i in range(0, self.tableWidget.rowCount(), 2):
            item_num = int((i / 2) + 1)
            old_item_num_str = self.tableWidget.item(i, 0).text()
            self.add_one_line(i, 0, item_num)
            if old_item_num_str in self.pulse_dict:
                new_pulse_dict[str(item_num)] = self.pulse_dict[old_item_num_str]
        self.pulse_dict = new_pulse_dict
        
        self.not_move = False

def run_ros_spin(node):
    """Run ROS2 spin in a separate thread."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f"ROS2 spin failed: {str(e)}")
    finally:
        if rclpy.ok() and node.is_valid():
            node.destroy_node()
        rclpy.try_shutdown()

def main():
    app = QApplication(sys.argv)
    node = None
    try:
        node = CalibrationWindow()
        node.show()
    except Exception as e:
        print(f"Failed to create or display Qt window: {str(e)}")
        if rclpy.ok():
            rclpy.try_shutdown()
        sys.exit(1)

    ros_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
