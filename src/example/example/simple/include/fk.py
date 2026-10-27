#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class FkDemo(Node):
    def __init__(self):
        super().__init__('fk_demo')  # Initialize the node (初始化节点)
        self.fk = ForwardKinematics(debug=True)

        # Create a bus servo publisher (创建总线舵机发布者)
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        # Wait for the underlying control service of the robotic arm to start (等待机械臂底层控制服务启动)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        # Servo pulse values (5 servos) (舵机脉宽值（5个舵机）)
        self.servo_list = [500, 600, 825, 110, 500]  # Corresponding IDs are 6, 5, 4, 3, 2 (对应ID依次为 6,5,4,3,2)
        self.duration = 1.0
        self.get_logger().info(f"舵机脉宽值：{self.servo_list}")

        # Convert pulse values to radians (脉宽值转为弧度)
        pulse = transform.pulse2angle(self.servo_list)
        self.get_logger().info(f"舵机脉宽值转为弧度：{pulse}")

        # Get forward kinematics result (获取正运动学解)
        res = self.fk.get_fk(pulse)
        self.get_logger().info(f'正运动学解-坐标：{res[0]}')
        self.get_logger().info(f'正运动学解-四元数：{res[1]}')
        self.get_logger().info(f'转换四元数成rpy：{transform.qua2rpy(res[1])}')

        # Publish servo information (in the order 6, 5, 4, 3, 2) (发布舵机信息（按 6,5,4,3,2 顺序）)
        self.set_servo_position(self.servos_pub, self.duration, self.servo_list)

    def set_servo_position(self, pub, duration, positions):
        """Publish servo positions in the order of IDs 6, 5, 4, 3, 2 (按 6,5,4,3,2 的舵机ID顺序发布位置)"""
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []

        servo_ids = [6, 5, 4, 3, 2]  # Fixed ID order (固定ID顺序)
        for idx, servo_id in enumerate(servo_ids):
            position = ServoPosition()
            position.id = servo_id
            position.position = float(positions[idx])
            position_list.append(position)

        msg.position = position_list
        msg.position_unit = "pulse"
        pub.publish(msg)
        self.get_logger().info(f"已发布舵机脉宽值（ID顺序 {servo_ids}）：{positions}")


def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy (初始化 rclpy)
    fk_demo_node = FkDemo()  # Create node (创建节点)
    rclpy.spin(fk_demo_node)  # Keep node running (保持节点运行)
    fk_demo_node.destroy_node()  # Destroy node (销毁节点)
    rclpy.shutdown()  # Shut down rclpy (关闭 rclpy)


if __name__ == '__main__':
    main()
