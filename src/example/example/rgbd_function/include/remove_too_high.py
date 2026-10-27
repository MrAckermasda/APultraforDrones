#!/usr/bin/python3
# coding=utf8
"""
Maintain equal height (等高保持)
Find the tallest object in the recognition area (找出识别区域高度最高的物体)
If the object height exceeds the threshold (如果物体高度超过阈值)
Then remove the tallest object (则将最高的物体移除)
Program testing uses 30x30x30mm and 40x40x40mm wooden blocks (程序测试使用的是 30x30x30mm的木块和 40x40x40mm的木块)
Button instructions (按键说明)：
    s - Start the game (开始玩法)
    a - Stop the game (停止玩法)
    ESC - Exit the program (退出程序)
"""

import cv2
import math
import time
import rclpy
import queue
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    """
    Convert depth pixel coordinates to XYZ in camera coordinate system (unit: meters) (将深度像素坐标转换为相机坐标系下的 XYZ（单位：米）)
    intrinsics: (fx, fy, cx, cy)
    """
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class RemoveTooHighObjectNode(Node):

    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.01],
        [0.0, -1.0, 0.0, 0.05],
        [0.0, 0.0, 0.0, 1.0]
    ]

    # Add pick_offset: x1, x2, y1, y2, z (添加 pick_offset: x1, x2, y1, y2, z)
    pick_offset = [-0.02, 0.01, 0.0, -0.02, -0.01]

    def __init__(self, name='remove_too_high'):
        # Initialize ROS (preserve original style) (初始化 ROS（保留原有风格）)
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # State variables (状态变量)
        self.endpoint = None
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.moving = False
        self.running = True
        self.stamp = time.time()
        self.count = 0
        self.last_position = (0, 0, 0)
        self.start_process = False   # Key-controlled start flag (按键控制的启动标志)
        self.last_pitch_yaw = (0, 0)

        # Message queues and publishers (消息队列与发布器)
        self.image_queue = queue.Queue(maxsize=2)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        # Clients (kinematics) (客户端（运动学）)
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.get_current_pose_client.wait_for_service()
        self.set_pose_target_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.set_pose_target_client.wait_for_service()

        # Services: keep but synchronize keyboard control (also controllable via service) (服务：保留但同步键盘控制（也可以通过 service 控制）)
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # Subscribe RGB, Depth, CameraInfo (synchronized) (订阅 RGB、Depth、CameraInfo（同步）)
        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.08)
        sync.registerCallback(self.multi_callback)

        # Start initialization timer (one-time) (启动初始化定时器（一次性）)
        timer_cb_group2 = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group2)

        self.get_logger().info("节点初始化完成，等待输入（按 s 开始，按 a 停止，ESC 退出）")

    def init_process(self):
        # Execute only once (只执行一次)
        try:
            self.timer.cancel()
        except Exception:
            pass

        # Move initial servos to safe position (初始舵机到位（安全位置）)
        try:
            set_servo_position(self.joints_pub, 1, ((6, 500), (5, 600), (4, 825), (3, 110), (2, 500), (1, 210)))
            time.sleep(2)
        except Exception as e:
            self.get_logger().info("设置初始舵机位置出错: " + str(e))

        # Start main thread (启动主线程)
        threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info("主线程已启动")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum=None, frame=None):
        self.running = False

    def start_srv_callback(self, request, response):
        # Service start also synchronizes gameplay (服务启动也会同步开始玩法)
        self.start_process = True
        response.success = True
        response.message = "start"
        self.get_logger().info("通过 ROS Service 启动玩法（等同按下 s）")
        return response

    def stop_srv_callback(self, request, response):
        # Service stop also synchronizes stop gameplay (服务停止也会同步停止玩法)
        self.start_process = False
        self.moving = False
        self.count = 0
        self.last_position = (0, 0, 0)
        # Move robot to safe pose (customizable) (将机械臂移动到安全姿态（可自定义）)
        try:
            set_servo_position(self.joints_pub, 1, ((6, 500), (5, 600), (4, 785), (3, 110), (2, 500), (1, 210)))
        except Exception as e:
            self.get_logger().info("停止时设置舵机位置出错: " + str(e))
        response.success = True
        response.message = "stop"
        self.get_logger().info("通过 ROS Service 停止玩法（等同按下 a）")
        return response

    def send_request(self, client, msg):
        """
        Send asynchronous request and wait for response (simple wrapper) (发送异步请求并等待返回（简单封装）)
        """
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    result = future.result()
                    return result
                except Exception as e:
                    self.get_logger().info("service 调用失败: " + str(e))
                    return None

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        # Put synchronized images into queue (pop oldest if full) (将同步得到的图像放入队列（若满则弹出最旧一帧）)
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except Exception:
                pass
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        """
        Get current robot end-effector pose and cache matrix (获取当前机械臂末端位姿并缓存矩阵)
        """
        res = self.send_request(self.get_current_pose_client, GetRobotPose.Request())
        if res is None:
            return None
        endpoint = res.pose
        self.endpoint = common.xyz_quat_to_mat(
            [endpoint.position.x, endpoint.position.y, endpoint.position.z],
            [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z]
        )
        return self.endpoint

    def pick(self, position, angle):
        """
        Pick process: check for possible stop requests during picking (if 'a' pressed, attempt to abort and return to safe position) (抓取流程：对抓取流程中可能的停止请求做检查（如果中途按下 a，会尽量中断并返回安全位）)
        position: world coordinates (x,y,z) (世界坐标（x,y,z）)
        angle: minimum bounding rectangle angle of object (物体最小外接矩形的角度)
        """
        # Normalize angle (规范化角度)
        angle = angle % 90
        angle = angle - 90 if angle > 40 else (angle + 90 if angle < -45 else angle)
        yaw = 80

        # Buzzer alert (optional) (蜂鸣提醒（可选）)
        try:
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.2
            msg.off_time = 0.01
            msg.repeat = 1
            # self.buzzer_pub.publish(msg)
        except Exception:
            pass

        # Apply pick_offset (based on conditions x>0.21, y>0) (应用 pick_offset（根据 x>0.21, y>0 等条件）)
        if position[0] > 0.21:
            offset_x = self.pick_offset[0]
        else:
            offset_x = self.pick_offset[1]
        if position[1] > 0:
            offset_y = self.pick_offset[2]
        else:
            offset_y = self.pick_offset[3]
        offset_z = self.pick_offset[-1]

        position = list(position)  # Ensure mutable (确保可变)
        position[0] += offset_x
        position[1] += offset_y
        position[2] += offset_z

        # Check for user stop at each step (start_process) (在每一步动作前检查是否被用户停止（start_process）)
        def abort_if_stopped():
            if not self.start_process:
                self.get_logger().info("检测到停止命令，终止当前抓取并返回安全位")
                self.moving = False
                try:
                    # Return to safe position (回安全位)
                    set_servo_position(self.joints_pub, 1, ((6, 500), (5, 600), (4, 815), (3, 110), (2, 500), (1, 210)))
                except Exception:
                    pass
                return True
            return False

        # Start a series of movements, checking after each step (开始一系列运动（每步后检查）)
        try:
            # First pose planning and movement (第一次位姿规划并移动)
            if abort_if_stopped():
                return
            msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res and res.pulse:
                servo_data = res.pulse
                set_servo_position(self.joints_pub, 1.5, ((6, servo_data[0]), (5, servo_data[1]), (4, servo_data[2]), (3, servo_data[3])))
                time.sleep(1.5)
            if abort_if_stopped():
                return

            # Control gripper angle (控制抓手角度)
            angle_val = 500 + int(1000 * (angle + (res.rpy[-1] if res and hasattr(res, 'rpy') else 0)) / 240)
            set_servo_position(self.joints_pub, 0.5, ((2, angle_val),))
            time.sleep(0.5)
            if abort_if_stopped():
                return

            # Refine pose again (再次微调位姿)
            msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res and res.pulse:
                servo_data = res.pulse
                set_servo_position(self.joints_pub, 1.5, ((6, servo_data[0]), (5, servo_data[1]), (4, servo_data[2]), (3, servo_data[3])))
                time.sleep(1.5)
            if abort_if_stopped():
                return

            # Close gripper (pseudo code: pulse value depends on hardware) (闭合抓手（伪代码：具体 pulse 值视硬件）)
            set_servo_position(self.joints_pub, 1.0, ((1, 600),))
            time.sleep(1)
            if abort_if_stopped():
                return

            # Reset and lift again (simulate placing action) (再复位与抬起（模拟放置动作）)
            msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res and res.pulse:
                servo_data = res.pulse
                set_servo_position(self.joints_pub, 1.5, ((6, servo_data[0]), (5, servo_data[1]), (4, servo_data[2]), (3, servo_data[3])))
                time.sleep(1.5)
            if abort_if_stopped():
                return

            # A series of pulses to move the robotic arm back to a safe position (values can be adjusted according to hardware) (一系列将机械臂移回安全位的脉冲（具体值可根据实际硬件调整）)
            set_servo_position(self.joints_pub, 1.0, ((6, 500), (5, 740), (4, 900), (3, 260), (2, 500)))
            time.sleep(1)
            if abort_if_stopped():
                return
            set_servo_position(self.joints_pub, 1.0, ((6, 150), (5, 635), (4, 900), (3, 260), (2, 500)))
            time.sleep(1)
            if abort_if_stopped():
                return
            set_servo_position(self.joints_pub, 1.0, ((6, 150), (5, 600), (4, 875), (3, 175), (2, 500)))
            time.sleep(1)
            if abort_if_stopped():
                return
            set_servo_position(self.joints_pub, 1.0, ((6, 150), (5, 600), (4, 875), (3, 175), (2, 500), (1, 200)))
            time.sleep(1)
            set_servo_position(self.joints_pub, 1, ((6, 500), (5, 600), (4, 815), (3, 110), (2, 500), (1, 210)))
            time.sleep(1)

        except Exception as e:
            self.get_logger().info("抓取过程中发生异常: " + str(e))
        finally:
            self.stamp = time.time()
            self.moving = False
            self.get_logger().info("抓取线程结束")

    def main(self):
        """
        Main loop: read images from queue, detect the highest object, control pick start/stop via key press (主循环：从队列中读取图像，检测最高物体，按键控制开始/停止抓取)
        """
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue

            try:
                # Convert ROS Image to numpy (转换 ROS Image 到 numpy)
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

                bgr_image = rgb_image    # Assume already BGR or RGB (depending on camera config) (假设已经是 BGR 或 RGB（根据相机配置）)
                K = depth_camera_info.k

                ih, iw = depth_image.shape[:2]
                depth = depth_image.copy()

                # Mask abnormal depth region near image bottom (original author's logic retained) (屏蔽靠近图像底部的异常深度区域（原作者逻辑保留）)
                if ih > 400:
                    depth[380:400, :] = np.array([[55555, ] * iw] * 20)

                # Replace too small depth values with a large number to avoid misidentifying nearest point (将过小深度替换为一个大值，避免被错判为最近点)
                depth_flat = depth.reshape((-1,)).copy()
                depth_flat[depth_flat <= 100] = 55555
                min_index = np.argmin(depth_flat)
                min_y = min_index // iw
                min_x = min_index - min_y * iw
                min_dist = depth_image[min_y, min_x]

                # Construct depth image for visualization (构造用于可视化的深度图)
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                # Set pixels farther than nearest point + 10mm to 0 (keep nearby objects) (将远于最近点 + 10mm 的像素置 0（保留近处物体）)
                depth_image = np.where(depth_image > min_dist + 10, 0, depth_image)
                sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                depth_gray = sim_depth_image_sort.astype(np.uint8)
                depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)
                _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
                depth_bit = cv2.erode(depth_bit, np.ones((5, 5), np.uint8))
                depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                # Find contours (找轮廓)
                contours, _ = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                largest = None
                z = 0
                position_text = ""

                for obj in contours:
                    area = cv2.contourArea(obj)
                    if area < 500 or self.moving:
                        continue
                    (cx, cy), radius = cv2.minEnclosingCircle(obj)
                    cv2.circle(depth_color_map, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)

                    # Get current robotic arm end-effector pose (获取机械臂当前末端位姿)
                    if self.get_endpoint() is None:
                        continue

                    # Check depth value (prevent out-of-bounds) (深度值检查（防止越界）)
                    cx_i = int(max(0, min(iw - 1, int(cx))))
                    cy_i = int(max(0, min(ih - 1, int(cy))))
                    depth_value = depth_image[cy_i, cx_i]
                    if depth_value == 0 or depth_value > 55554:
                        continue

                    # Convert camera pixel to camera coordinates (unit: meters) (相机像素转相机坐标（单位 m）)
                    position_cam = depth_pixel_to_camera((cx, cy), depth_value / 1000.0, (K[0], K[4], K[2], K[5]))
                    position_cam[2] = position_cam[2] + 0.03  # Raise offset (抬高偏移)
                    pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position_cam, (0, 0, 0)))
                    world_pose = np.matmul(self.endpoint, pose_end)
                    pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                    pose_t[1] += 0.01  # Fine-tune y（微调 y）

                    # Record the highest (max Z) object (记录最高（Z 最大）的物体)
                    if pose_t[2] > z:
                        largest = (obj, pose_t, (cx_i, cy_i))
                        z = pose_t[2]
                        # Display distance on depth map (在深度图显示距离)
                        txt = 'Dist: {}mm'.format(depth_value)
                        cv2.circle(depth_color_map, (cx_i, cy_i), 8, (32, 32, 32), -1)
                        cv2.circle(depth_color_map, (cx_i, cy_i), 6, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                        # Also display on color image (同时在彩色图显示)
                        cv2.circle(bgr_image, (cx_i, cy_i), 8, (32, 32, 32), -1)
                        cv2.circle(bgr_image, (cx_i, cy_i), 6, (255, 255, 255), -1)
                        cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                        cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                # If highest object found, display position info and trigger pick based on conditions (若找到最高物体，显示位置信息并根据条件触发抓取)
                if largest is not None:
                    obj, pose_t, (min_x, min_y) = largest
                    position_text = f"x:{pose_t[0]:.3f} m, y:{pose_t[1]:.3f} m, z:{pose_t[2]:.3f} m"
                    cv2.putText(bgr_image, position_text, (10, ih - 50), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                    cv2.putText(bgr_image, position_text, (10, ih - 50), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                    # Compute distance to last detected position (for jitter filtering) (计算与上一次检测到位置的距离（用于抖动过滤）)
                    dist = math.sqrt((self.last_position[0] - pose_t[0]) ** 2 + (self.last_position[1] - pose_t[1]) ** 2 + (self.last_position[2] - pose_t[2]) ** 2)
                    self.last_position = pose_t
                    self.get_logger().info(f"目标高度:z={pose_t[2]:.3f} m, 与上次偏差 dist={dist:.4f}")

                    # Trigger pick: only pick if start_process=True and target is stable (触发抓取：只有在 start_process=True 且目标稳定时才抓取)
                    if dist < 0.002 and 0.01 < pose_t[2] and self.start_process:
                        if time.time() - self.stamp > 0.5:
                            self.stamp = time.time()
                            rect = cv2.minAreaRect(obj)
                            self.moving = True
                            # Use a thread to pick so as not to block the main loop (使用线程抓取以不阻塞主循环)
                            threading.Thread(target=self.pick, args=(pose_t, rect[2]), daemon=True).start()
                    else:
                        self.stamp = time.time()

                # Update FPS (if display needed) (更新 FPS（如果需要显示）)
                self.fps.update()
                result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                cv2.imshow("depth", result_image)

                # ===== Keyboard listener (main control: s start, a stop, ESC exit) (键盘监听（主控：s 开始，a 停止，ESC 退出）) =====
                key = cv2.waitKey(1)
                if key == ord('s'):
                    if not self.start_process:
                        self.start_process = True
                        self.get_logger().info("用户按下 's'：开始玩法（按 a 停止）")
                elif key == ord('a'):
                    if self.start_process:
                        self.start_process = False
                        self.moving = False
                        self.count = 0
                        self.last_position = (0, 0, 0)
                        # Optionally return to safe position (可选回到安全位)
                        try:
                            set_servo_position(self.joints_pub, 1, ((6, 500), (5, 600), (4, 785), (3, 110), (2, 500), (1, 210)))
                        except Exception:
                            pass
                        self.get_logger().info("用户按下 'a'：停止玩法（按 s 重新开始）")
                elif key == 27:  # ESC Exit(退出)
                    self.get_logger().info("用户按下 ESC：退出程序")
                    self.running = False
                    break

            except Exception as e:
                # Handle main loop exception, prevent thread exit (处理主循环异常，不让线程退出)
                self.get_logger().info('处理异常: ' + str(e))

        # Exit and cleanup (退出清理)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.get_logger().info("节点已退出")

def main():
    node = RemoveTooHighObjectNode('remove_too_high')
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt，退出中...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == "__main__":
    main()
