# camera_thread.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtCore import pyqtSignal, QObject
import cv2
import numpy as np

class ROS_Camera(Node, QObject):
    raw_data = pyqtSignal(np.ndarray)    # Define signal to pass image data (定义信号，用于传递图像数据)

    def __init__(self, topic: str):
        Node.__init__(self, f"camera_subscriber_{topic.replace('/', '_')}")
        QObject.__init__(self)
        self.topic = topic
        self.bridge = CvBridge()
        self.subscription = None
        self.running = False

    def image_callback(self, msg: Image):
        """
        Image topic callback function, convert ROS image to OpenCV format and emit signal
        (图像话题回调函数，将 ROS 图像转换为 OpenCV 格式并发送信号)
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.raw_data.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {str(e)}")

    def enter(self):
        """
        Initialize image subscription (初始化图像订阅)
        """
        if self.subscription is None:
            self.subscription = self.create_subscription(
                Image,
                self.topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f"订阅图像话题: {self.topic}")

    def start(self):
        """
        Start subscription (开始订阅)
        """
        self.running = True
        self.get_logger().info(f"相机线程启动: {self.topic}")

    def exit(self):
        """
        Stop subscription and clean up (停止订阅并清理)
        """
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self.running = False
        self.get_logger().info(f"相机线程退出: {self.topic}")
