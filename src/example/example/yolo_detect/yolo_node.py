#!/usr/bin/python3
#coding=utf8
#Author: gcusms
#Date: 2026-01-20

from sdk.paths import workspace_path
import cv2
import os
import time
import math
import queue
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ObjectInfo, ObjectsInfo
from ultralytics import YOLO

# Environment variable optimization (环境变量优化)
os.environ["OV_CPU_DISABLE_WEIGHTS_CACHE"] = "1"

class Colors:
    def __init__(self):
        hex_colors = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
                      '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex_colors]
        self.n = len(self.palette)
    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c
    @staticmethod
    def hex2rgb(h):
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()

class yoloNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        def get_param(p_name, default):
            if not self.has_parameter(p_name):
                self.declare_parameter(p_name, default)
            return self.get_parameter(p_name).value

        # --- Parameter Acquisition (参数获取) ---
        self.start = bool(get_param('start', False))
        self.conf = float(get_param('conf', 0.5))
        self.classes = get_param('classes', ['object'])
        self.task = str(get_param('task', 'detect')) 
        self.display = bool(get_param('display', False))
        self.model_name = get_param('model', 'best_garbage_26')
        self.image_topic = get_param('image_topic', '/depth_cam/rgb/image_raw')
        self.model_size = int(get_param('model_size', 320))

        self.running = True
        
        self.prev_time = time.time()
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        signal.signal(signal.SIGINT, self.shutdown)

        # --- Load Model (加载模型) ---
        if '11' in self.model_name:
            model_path = workspace_path(f'src/example/example/yolo_detect/model/{self.model_name}_openvino_model/')
        if '26' in self.model_name:
            model_path = workspace_path(f'src/example/example/yolo_detect/model/{self.model_name}_openvino_model/')
        self.model = YOLO(model_path, task=self.task)

        # --- ROS 2 Interfaces (ROS 2 接口) ---
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.get_logger().info(f"YOLO Node [Task:{self.task}] started")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.start = True
        response.success = True
        return response

    def stop_srv_callback(self, request, response):
        self.start = False
        response.success = True
        return response

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            if self.image_queue.full(): self.image_queue.get()
            self.image_queue.put(cv_image)
        except Exception: pass
   
    def shutdown(self, signum, frame):
        self.running = False

    def image_proc(self):
        self.fps = 0.0
        self.alpha = 0.1
        while self.running:
            try:
                img = self.image_queue.get(timeout=1)
            except queue.Empty: continue
            start_time = time.time()
            if not self.start:
                self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                continue
            try:
                # --- Inference (推理) ---
                results = self.model(img, device="CPU", imgsz=self.model_size, conf=self.conf, iou=0.45, verbose=False)

                objects_info = []
                annotated_frame = img.copy()

                for r in results:
                    annotated_frame = r.plot()

                    # 1. Handling Oriented Bounding Boxes (处理旋转框 OBB)
                    if self.task == 'obb' and r.obb is not None:
                        for i in range(len(r.obb.cls)):
                            obj = ObjectInfo()
                            cls_id = int(r.obb.cls[i])
                            obj.class_name = self.classes[cls_id] if cls_id < len(self.classes) else str(cls_id)
                            
                            # Get OBB data: [cx, cy, w, h, rotation] (获取 OBB 数据: [cx, cy, w, h, rotation])
                            data = r.obb.xywhr[i].cpu().numpy()
                            cx, cy, w, h, rotation = data[0], data[1], data[2], data[3], data[4]

                            # Adapt receiver center calculation: (box[0]+box[2])/2 = cx (适配接收端中心点计算: (box[0]+box[2])/2 = cx)
                            obj.box = [int(cx), int(cy), int(cx), int(cy)]
                            
                            obj.angle = int(math.degrees(rotation))
                            obj.score = float(r.obb.conf[i])
                            obj.width, obj.height = int(w), int(h)
                            objects_info.append(obj)

                    # 2. Handling Standard Bounding Boxes (处理标准框 Detect)
                    elif r.boxes is not None:
                        for i in range(len(r.boxes.cls)):
                            obj = ObjectInfo()
                            cls_id = int(r.boxes.cls[i])
                            obj.class_name = self.classes[cls_id] if cls_id < len(self.classes) else str(cls_id)
                            
                            # Standard mode uses [x1, y1, x2, y2] (标准模式使用 [x1, y1, x2, y2])
                            box = r.boxes.xyxy[i].cpu().numpy()
                            obj.box = [int(box[0]), int(box[1]), int(box[2]), int(box[3])]
                            
                            obj.angle = 0
                            obj.score = float(r.boxes.conf[i])
                            obj.width = int(box[2] - box[0])
                            obj.height = int(box[3] - box[1])
                            objects_info.append(obj)

                # Publish ObjectsInfo (发布 ObjectsInfo)
                msg = ObjectsInfo()
                msg.objects = objects_info
                self.object_pub.publish(msg)

                # FPS Calculation (FPS 计算与显示)
                end_time = time.time()
                duration = end_time - start_time
                
                if duration > 0:
                    current_fps = 1.0 / duration
                    self.fps = (self.alpha * current_fps) + (1 - self.alpha) * self.fps
            
                cv2.putText(annotated_frame, f"FPS: {self.fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if self.display:
                    cv2.imshow("yolo_detect_node", annotated_frame)
                    if cv2.waitKey(1) == 27: break
                
                self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

            except Exception as e:
                self.get_logger().error(f"Detection Error: {e}")

def main():
    rclpy.init()
    node = yoloNode('yolo')
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
