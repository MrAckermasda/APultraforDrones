#!/usr/bin/env python3
# encoding: utf-8
# Inverse kinematics control example (逆运动学控制例程)
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class IkDemo(Node):
    def __init__(self):
        super().__init__('ik_demo_node')

        self.coordinate = [0.16996271619072376, 0.0, 0.2915247886198293]
        self.servo_list = [] # Store pulse values (存储脉宽值)
        self.duration = 1.0

        # Publish bus servo topic (发布总线舵机话题)
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        # Wait for the underlying control service of the robotic arm to start (等待机械臂底层控制服务启动)
        self._wait_for_services()

        # Obtain and process the inverse kinematics solution (获取并处理逆运动学解)
        self._process_ik_solution()

    def _wait_for_services(self):
        """Wait for ROS services to be ready (等待 ROS 服务就绪)"""
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

    def _process_ik_solution(self):
        """Obtain inverse kinematics solution and convert it to servo pulse values (获取逆运动学解并转换为舵机脉宽值)"""
        self.get_logger().info(f"末端坐标：{self.coordinate}")
        res = get_ik(self.coordinate, 90, [-180, 180])

        if res:
            for i, solution in enumerate(res):
                self.get_logger().info(f'rpy{i + 1}: {solution[1]}')
                pulse = transform.angle2pulse(solution[0])  # Convert angles to pulse values (将角度转换为脉宽值)
                self.servo_list = pulse[0]

                for j, p in enumerate(pulse):
                    self.get_logger().info(f'舵机脉宽值{j + 1}: {p}')

            # Publish servo positions (in order 6→5→4→3→2) (发布舵机位置（按6→5→4→3→2的顺序）)
            self.set_servo_position(self.servos_pub, self.duration, self.servo_list)
        else:
            self.get_logger().warning("没有逆运动学解，请检查末端坐标")

    def set_servo_position(self, pub, duration, positions):
        """Publish servo position commands (servo IDs in order 6,5,4,3,2) (发布舵机位置指令（舵机ID按 6,5,4,3,2 顺序）)"""
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []

        servo_ids = [6, 5, 4, 3, 2]  # Target servo order (目标舵机顺序)
        for idx, servo_id in enumerate(servo_ids):
            position = ServoPosition()
            position.id = servo_id
            position.position = float(positions[idx])  # Corresponding pulse value (对应脉宽值)
            position_list.append(position)

        msg.position = position_list
        msg.position_unit = "pulse"
        pub.publish(msg)
        self.get_logger().info(f"已发布舵机脉宽值（ID顺序 {servo_ids}）：{positions}")

def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy (初始化 rclpy)
    ik_demo_node = IkDemo()  # Create node (创建节点)
    rclpy.spin(ik_demo_node)  # Keep node running (保持节点运行)
    ik_demo_node.destroy_node()  # Destroy node (销毁节点)
    rclpy.shutdown()  # Shut down rclpy (关闭 rclpy)

if __name__ == '__main__':
    main()
