import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl  

class MecanumChassis(Node):
    def __init__(self, wheelbase=0.110, track_width=0.0975, wheel_diameter=0.0965):
        super().__init__('mecanum_chassis')

        # 麦克纳姆轮的物理参数
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(MotorsSpeedControl, '/ros_robot_controller/set_motor_speeds', 10)

    def speed_convert(self, speed):
        """
        将速度转换为电机速度比例，取消最大速度限制。
        :param speed: 速度（m/s）
        :return: 电机控制速度（百分比）
        """
        motor_speed = speed * 100  
        return motor_speed

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        motor_msg = self.set_velocity(linear_x, linear_y, angular_z)
        self.publisher.publish(motor_msg)

    def set_velocity(self, linear_x, linear_y, angular_z):
        l_plus_w = (self.wheelbase + self.track_width) / 2
        
        v_left_front = linear_x - linear_y - angular_z * l_plus_w
        v_right_front = linear_x + linear_y + angular_z * l_plus_w
        v_left_rear = linear_x + linear_y - angular_z * l_plus_w
        v_right_rear = linear_x - linear_y + angular_z * l_plus_w

        motor_values = [v_right_front, v_right_rear, v_left_front, v_left_rear]

        invert_motor = {
            1: False,
            2: True,
            3: True,
            4: False,
        }

        motor_speeds = []
        for idx, val in enumerate(motor_values, start=1):
            if invert_motor[idx]:
                val = -val
            motor_speed = self.speed_convert(val)
            motor_speeds.append(motor_speed)

        msg = MotorsSpeedControl()
        msg.data = []
        for i in range(4):
            motor_msg = MotorSpeedControl()
            motor_msg.id = i + 1
            motor_msg.speed = float(motor_speeds[i])
            msg.data.append(motor_msg)
        return msg


def main(args=None):
    rclpy.init(args=args)

    mecanum_chassis = MecanumChassis()
    rclpy.spin(mecanum_chassis)
    mecanum_chassis.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()