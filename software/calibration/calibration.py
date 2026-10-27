#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18

from pathlib import Path
import os
import cv2
import math
import time
import yaml
import queue
import threading
import numpy as np

import rclpy
import message_filters
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener, TransformException

import sdk.fps as fps
from sdk import common
from app.utils import image_process
from app import calibration
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from kinematics.kinematics_control import set_pose_target, set_joint_value_target
from app.utils import utils, calculate_grasp_yaw_by_depth, position_change_detect, pick_and_place, distortion_inverse_map

class ColorPicker:
    def __init__(self, point, repeat):
        self.point = point
        self.count = 0
        self.color = []
        self.rgb = []
        self.repeat = repeat

    def set_point(self, point):
        self.point = point

    def reset(self):
        self.count = 0
        self.color = []
        self.rgb = []

    def __call__(self, image, result_image):
        h, w = image.shape[:2]
        x, y = self.point[0], self.point[1]
        if y == 0:
            y = 1
        if y == h:
            y = h - 1
        if x == 0:
            x = 1
        if x == w:
            x = w - 1
        image_lab = cv2.cvtColor(image[y - 1:y + 1, x - 1:x + 1], cv2.COLOR_BGR2LAB)
        self.color.extend(image_lab.tolist())
        self.rgb.extend(image[y - 1:y + 1, x - 1:x + 1].tolist())
        self.count += 1
        l, a, b = 0, 0, 0
        r, g, b_ = 0, 0, 0
        for c in self.color:
            l, a, b = l + c[0][0] + c[1][0], a + c[0][1] + c[1][1], b + c[0][2] + c[1][2]
        for c in self.rgb:
            r, g, b_ = r + c[0][0] + c[1][0], g + c[0][1] + c[1][1], b_ + c[0][2] + c[1][2]
        l, a, b = int(l / (2 * len(self.color))), int(a / (2 * len(self.color))), int(b / (2 * len(self.color)))
        r, g, b_ = int(r / (2 * len(self.rgb))), int(g / (2 * len(self.rgb))), int(b_ / (2 * len(self.rgb)))
        if 0 <= x < w and 0 <= y < h:
            result_image = cv2.circle(result_image, (x, y), self.count, (r, g, b_), 2 * self.count)
            result_image = cv2.circle(result_image, (x, y), self.count, (255, 255, 0), 5)
        if len(self.color) / 2 > self.repeat:
            self.color.remove(self.color[0])
            self.color.remove(self.color[0])
        if len(self.rgb) / 2 > self.repeat:
            self.rgb.remove(self.rgb[0])
            self.rgb.remove(self.rgb[0])
        if self.count > self.repeat:
            self.count = self.repeat
        if self.count >= self.repeat:
            return ((l, a, b), (r, g, b_)), result_image
        else:
            return None, result_image

class ColorPick(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS()
        self.rgb_image_queue = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_type = os.environ['CAMERA_TYPE']
        self.running = True
        self.center = []
        self.endpoint = None
        self.imgpts = None
        self.target_color = None
        self.set_callback = False
        self.color_picker = None
        self.intrinsic = None
        self.distortion = None
        self.transport_info = None
        self.count_move = 0
        self.count_still = 0
        self.mode = 'color'
        self.calibration = False
        self.depth_enable = False
        self.last_position = None
        self.start_transport = False
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.image_process = image_process.GetObjectSurface(100, 200)
        self.timer_cb_group = ReentrantCallbackGroup()
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.set_joint_value_target_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target',
                                                                callback_group=self.timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()
        self.create_service(SetBool, 'calibration/start_calibration', self.start_calibration_srv_callback)
        
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.camera_info_file = str(Path(__file__).resolve().parents[2] / 'src/peripherals/config/camera_info.yaml')

        self.chassis_type = os.environ['CHASSIS_TYPE']
        if self.chassis_type == 'Slide_Rails':
            self.config_path = str(Path(__file__).resolve().parents[2] / "src/example/example/stepper/config/") + '/'
        else:
            self.config_path = str(Path(__file__).resolve().parents[2] / "src/app/config/") + '/'
        
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            self.plane = config['plane']
            self.corners = np.array(config['corners'])
            self.extristric = np.array(config['extristric'])
            self.white_area_center = np.array(config['white_area_pose_world']).reshape(4, 4)
        
        with open(self.camera_info_file, 'r') as f:
            cam_config = yaml.safe_load(f)
            self.hand2cam_tf_matrix = np.array(cam_config['hand2cam_tf_matrix'])

        self.rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/rgb/camera_info')
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.info_sub], 3, 0.2)
        self.sync.registerCallback(self.rgb_callback)

        if self.camera_type == 'aurora':
            self.depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
            self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
            self.sync_depth = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.depth_info_sub], 3, 0.2)
            self.sync_depth.registerCallback(self.depth_callback)

        self.create_subscription(Image, '/calibration/image_result', self.image_callback, 1)
        self.create_subscription(Bool, '/calibration/finish', self.finish_calibration_callback, 1)

        tf_buffer = Buffer()
        if self.camera_type == 'aurora':
            self.tf_listener = TransformListener(tf_buffer, self)
            tf_future = tf_buffer.wait_for_transform_async(
                target_frame='depth_cam_depth_optical_frame',
                source_frame='depth_cam_color_frame',
                time=rclpy.time.Time()
            )
            print("正在等待变换...")
        else:
            self.tf_listener = TransformListener(tf_buffer, self)
            tf_future = tf_buffer.wait_for_transform_async(
                target_frame='depth_camera_link',
                source_frame='depth_cam_color_frame',
                time=rclpy.time.Time()
            )
        print("完成")
        rclpy.spin_until_future_complete(self, tf_future)
        try:
            transform = tf_buffer.lookup_transform(
                'depth_cam_color_frame', 'depth_camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0) )
            self.static_transform = transform
            self.get_logger().info(f'静态变换: {self.static_transform}')
        except TransformException as e:
            self.get_logger().error(f'获取静态变换失败: {e}')
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_matrix = common.xyz_quat_to_mat([translation.x, translation.y, translation.z], [rotation.w, rotation.x, rotation.y, rotation.z])
        self.hand2cam_tf_matrix = np.matmul(transform_matrix, self.hand2cam_tf_matrix)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def init_process(self):
        self.timer.cancel()
        self.go_home()
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.pick_and_place_thread, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % '初始化完成')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"鼠标点击位置 ({x}, {y})")
            with self.lock:
                if x > 640:
                    self.mode = 'depth'
                    self.center = [x - 640, y]
                else:
                    self.mode = 'color'
                    if self.camera_type == 'usb_cam':
                        self.color_picker = ColorPicker([x, y + 40], 20)
                        self.center = [x, y + 40]
                        self.center = [x, y]
                        self.target_color = None
                        self.get_logger().info(f'{x}, {y}')
                    else:
                        self.color_picker = ColorPicker([x, y], 20)
                        self.center = [x, y]
                        self.target_color = None
                        self.get_logger().info(f'{x}, {y}')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def go_home(self):
        joint_angle = [500, 600, 825, 110, 500, 210]
        msg = set_joint_value_target(joint_angle)
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        with self.lock:
            self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z],
                                                   [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        set_servo_position(self.joints_pub, 1, ((5, joint_angle[1]), (4, joint_angle[2]), (3, joint_angle[3]), (2, 500),(6, 500)))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1, ((6, joint_angle[0]), (1, 200)))
        self.get_logger().info(f"关节角度[ID5]: {joint_angle[1]}")
        self.get_logger().info(f"关节角度[ID4]: {joint_angle[2]}")
        self.get_logger().info(f"关节角度[ID3]: {joint_angle[3]}")
        time.sleep(1)

    def start_calibration_srv_callback(self, request, response):
        with self.lock:
            self.calibration = request.data
        response.success = True
        response.message = "开始"
        return response

    def calculate_pick_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        angle1 = angle
        angle2 = angle - 90
        yaw1 = yaw + angle1
        if yaw1 < 0:
            yaw2 = yaw1 + 90
        else:
            yaw2 = yaw1 - 90
        if abs(yaw1) < abs(yaw2):
            yaw = yaw1
        else:
            yaw = yaw2
        yaw = 500 + int(yaw / 240 * 1000)
        return yaw

    def get_imgpts(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)
        tvec = extristric[:1]
        rmat = extristric[1:]
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

    def get_pixel_position(self, bgr_image, display_image, point):
        if self.color_picker is not None and self.target_color is None:
            self.target_color, display_image = self.color_picker(bgr_image, display_image)
        elif self.target_color is not None and point:
            roi_img = self.image_process.get_top_surface(bgr_image)
            image_lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
            threshold = 0.3
            min_color = [int(self.target_color[0][0] - 50 * threshold * 2),
                         int(self.target_color[0][1] - 50 * threshold),
                         int(self.target_color[0][2] - 50 * threshold)]
            max_color = [int(self.target_color[0][0] + 50 * threshold * 2),
                         int(self.target_color[0][1] + 50 * threshold),
                         int(self.target_color[0][2] + 50 * threshold)]
            mask = cv2.inRange(image_lab, tuple(min_color), tuple(max_color))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)
            contours = map(lambda a_c: a_c[1], filter(lambda a: 500 <= a[0] <= 7000, contours_area))
            min_d = float('inf')
            target = None
            for c in contours:
                rect = cv2.minAreaRect(c)
                d = math.sqrt((rect[0][0] - point[0]) ** 2 + (rect[0][1] - point[1]) ** 2)
                if d < min_d:
                    min_d = d
                    target = rect
            self.color_picker = None
            cv2.circle(display_image, (int(target[0][0]), int(target[0][1])), 8, (0, 255, 255), -1)
            box = np.intp(cv2.boxPoints(target))
            cv2.drawContours(display_image, [box], -1, (0, 255, 255), 2, cv2.LINE_AA)
            if self.camera_type == 'usb_cam':
                    x, y = distortion_inverse_map.undistorted_to_distorted_pixel(target[0][0], target[0][1], self.intrinsic, self.distortion)
                    target = ((x, y), target[1], target[-1])
            position, projection_matrix = self.get_object_world_position(target[0], self.intrinsic)
            yaw = self.calculate_pick_grasp_yaw(position, target[-1])
            return [position, yaw]
        return None

    def get_object_world_position(self, position, intrinsic, height=0.03):
        tvec = self.extristric[:1]
        rmat = self.extristric[1:]
        self.get_logger().info(f'{position}')
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        projection_matrix = np.row_stack((np.column_stack((rmat, tvec)), np.array([[0, 0, 0, 1]])))
        world_pose = common.pixels_to_world([position], intrinsic, projection_matrix)[0]
        world_pose[0] = -world_pose[0]
        world_pose[1] = -world_pose[1]
        position = self.white_area_center[:3, 3] + world_pose
        position[2] = height
        self.get_logger().info(f'位置1:{position}')
        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['pixel']['offset'])
        scale = tuple(config_data['pixel']['scale'])
        for i in range(3):
            position[i] = position[i] * scale[i]
            position[i] = position[i] + offset[i]
        return position, projection_matrix

    def get_object_position(self, depth_image, depth_color_map, bgr_image, camera_info, depth_camera_info, point):
        if point:
            h, w = depth_image.shape[:2]
            roi_h, roi_w = 5, 5
            w_1 = point[0] - roi_w
            w_2 = point[0] + roi_w
            if w_1 < 0:
                w_1 = 0
            if w_2 > w:
                w_2 = w
            h_1 = point[1] - roi_h
            h_2 = point[1] + roi_h
            if h_1 < 0:
                h_1 = 0
            if h_2 > h:
                h_2 = h
            w_1 = int(w_1)
            w_2 = int(w_2)
            h_1 = int(h_1)
            h_2 = int(h_2)
            cv2.rectangle(depth_color_map, (w_1, h_1), (w_2, h_2), (0, 255, 0), 2)
            roi = depth_image[h_1:h_2, w_1:w_2]
            distances = roi[np.logical_and(roi > 0, roi < 40000)]
            if len(distances) > 0:
                distance = int(np.mean(distances))
                plane_values = utils.get_plane_values(depth_image, self.plane, depth_camera_info.k)
                contours = utils.extract_contours(plane_values, 0.015)
                min_distance = float('inf')
                center = None
                for cnt in contours:
                    (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                    distance = np.sqrt((cx - point[0]) ** 2 + (cy - point[1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        center = [cnt, int(cx), int(cy)]
                if center is not None:
                    [x, y], (width, height), angle = cv2.minAreaRect(center[0])
                    x, y = int(x), int(y)
                    depth = depth_image[y, x]
                    self.get_logger().info(f'深度1:{depth}')
                    position = utils.calculate_world_position(x, y, depth, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_camera_info.k)
                    yaw = self.calculate_pick_grasp_yaw(position, angle)
                    self.get_logger().info(f'位置1:{position}')
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['depth']['offset'])
                    scale = tuple(config_data['depth']['scale'])
                    for i in range(3):
                        position[i] = position[i] * scale[i]
                        position[i] = position[i] + offset[i]
                    cv2.circle(depth_color_map, (x, y), 5, (0, 0, 255), -1)
                    cv2.drawContours(depth_color_map, [np.intp(cv2.boxPoints(((x, y), (width, height), angle)))], -1, (0, 0, 255), 2, cv2.LINE_AA)
                    return [position, yaw]
        return None

    def pick_and_place_thread(self):
        while self.running:
            if self.start_transport:
                position, yaw = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]
                self.get_logger().info(f'位置3:{position}')
                position[2] += 0.03
                msg = set_pose_target(position, 80, [-180.0, 180.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                if res.pulse:
                    servo_data = res.pulse
                    set_servo_position(self.joints_pub, 0.5, ((6, servo_data[0]),))
                    time.sleep(0.5)
                    set_servo_position(self.joints_pub, 0.8, ((5, servo_data[1]), (4, servo_data[2]), (3, servo_data[3])))
                    time.sleep(0.8)
                    set_servo_position(self.joints_pub, 0.3, ((2, yaw),))
                    time.sleep(0.3)
                    position[2] -= 0.03
                    position[2] -= 0.015
                    msg = set_pose_target(position, 80, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.kinematics_client, msg)
                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 0.5, ((5, servo_data[1]), (4, servo_data[2]), (3, servo_data[3])))
                        time.sleep(0.5)
                        set_servo_position(self.joints_pub, 0.3, ((1, 500),))
                        time.sleep(0.3)
                with self.lock:
                    self.center = []
                    self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while self.running:
            try:
                if self.calibration:
                    result_image = self.image_queue.get(block=True, timeout=10)
                else:
                    bgr_image, camera_info = self.rgb_image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            if not self.calibration:
                with self.lock:
                    self.intrinsic = np.matrix(camera_info.k).reshape(1, -1, 3)
                    self.distortion = np.array(camera_info.d)
                    if self.chassis_type == 'Slide_Rails':
                        self.get_imgpts()
                        cv2.drawContours(bgr_image, [self.imgpts], -1, (0, 255, 255), 2, cv2.LINE_AA)
                result_image = np.copy(bgr_image)
            depth_color_map = None
            if not self.start_transport:
                with self.lock:
                    position = None
                    if self.center and self.mode == 'color' and not self.calibration:
                        result = self.get_pixel_position(bgr_image, result_image, self.center)
                        if result is not None:
                            position, angle = result
                    if self.depth_enable:
                        try:
                            depth_image, depth_camera_info = self.depth_image_queue.get(block=True, timeout=1)
                        except queue.Empty:
                            continue
                        max_dist = 350
                        depth_image = utils.create_roi_mask(depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.05)
                        min_dist = utils.find_depth_range(depth_image, max_dist)
                        sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                        depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                        if self.center and self.mode == 'depth':
                            result = self.get_object_position(depth_image, depth_color_map, bgr_image, camera_info, depth_camera_info, self.center)
                            if result is not None:
                                position, angle = result
                    if self.last_position is not None and position is not None:
                        e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                            pow(self.last_position[1] - position[1], 2)), 5)
                        if e_distance <= 0.005:
                            self.count_move = 0
                            self.count_still += 1
                        else:
                            self.count_move += 1
                            self.count_still = 0
                        if self.count_move > 10:
                            self.count_move = 0
                        if self.count_still > 5:
                            self.count_still = 0
                            self.count_move = 0
                            self.get_logger().info(f'位置2:{position}')
                            self.transport_info = [position, angle]
                            self.start_transport = True
                    self.last_position = position
            self.fps.update()
            self.fps.show_fps(bgr_image)
            if self.depth_enable and depth_color_map is not None:
                result_image = np.concatenate([result_image, depth_color_map], axis=1)
            if self.camera_type == 'usb_cam':
                result_image = result_image[40:440, ]
            cv2.imshow('result_image', result_image)
            if not self.set_callback:
                self.set_callback = True
                cv2.setMouseCallback("result_image", self.mouse_callback)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def rgb_callback(self, ros_rgb_image, camera_info):
        cv_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.rgb_image_queue.full():
            self.rgb_image_queue.get()
        self.rgb_image_queue.put((bgr_image, camera_info))

    def depth_callback(self, ros_depth_image, depth_camera_info):
        self.depth_enable = True
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        if self.depth_image_queue.full():
            self.depth_image_queue.get()
        self.depth_image_queue.put((depth_image, depth_camera_info))

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(image)

    def finish_calibration_callback(self, msg):
        with self.lock:
            with open(self.config_path + self.config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.plane = config['plane']
                self.corners = np.array(config['corners'])
                self.extristric = np.array(config['extristric'])
                self.white_area_center = np.array(config['white_area_pose_world']).reshape(4, 4)

def main():
    node = ColorPick('color_pick')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
