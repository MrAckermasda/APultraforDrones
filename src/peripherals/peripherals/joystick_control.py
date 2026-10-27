#!/usr/bin/env python3
# coding=utf8

import math
import os
import threading
import pygame as pg
import rclpy
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from peripherals.jog_settings import JOINT_SETTINGS, validate_joint_setting

from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller import bus_servo_control, actions
import kinematics.transform as transform
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoStateList
from ros_robot_controller_msgs.msg import BuzzerState

###############################################################################
AXES_MAP =  ('0', '1', '2', '3', 'hat_x', 'hat_y')


BUTTONS = [("cross", "circle", "", "square","triangle", "", "l1",
           "r1", "l2", "r2", "select", "start", "mode","lc","rc"),
           
           ("triangle", "circle", "cross",  "square","l1", "r1", "l2", "r2", 
            "select", "start",  "lc","rc","mode","","")]


class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        # Validate all startup settings before publishers or initialization motion.
        self._declare_jog_parameters()
        self.enable_kinematics = self.declare_parameter(
            'enable_kinematics', False,
            descriptor=ParameterDescriptor(
                read_only=True, description='Enable Cartesian mode; requires working FK/IK services'),
        ).value
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pg.display.init()

        self.BUTTONS = BUTTONS[0]
        self.last_buttons = dict.fromkeys(self.BUTTONS, 0)

        # 模式: 0 单舵机, 1 逆运动学
        self.mode = 0
        self.pitch = None
        self.current_servo_position = None

        self.test = 0 

        # 发布器
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.servo_states_sub = self.create_subscription(ServoStateList, 'controller_manager/servo_states', self.servo_states_callback, 10)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)

        service_group = ReentrantCallbackGroup()
        self.get_current_pose_client = None
        self.kinematics_client = None
        if self.enable_kinematics:
            self.get_current_pose_client = self.create_client(
                GetRobotPose, '/kinematics/get_current_pose', callback_group=service_group)
            self.kinematics_client = self.create_client(
                SetRobotPose, '/kinematics/set_pose_target', callback_group=service_group)
        self.client = self.create_client(
            Trigger, '/controller_manager/init_finish', callback_group=service_group)
        while not self.client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                raise RuntimeError('Shutdown while waiting for controller_manager')
            self.get_logger().warning(
                'Waiting for /controller_manager/init_finish; start the robot driver '
                'and servo_controller (enable_grasp:=false for joint-only control).')

        # 手柄
        self.js = None
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))        
        self.last_buttons = [0] * len(BUTTONS[0])

        # SDL initialization, hotplug handling and input polling share the same
        # callback group; no forever-loop may occupy a timer callback.
        self.create_timer(0.1, self.update_buttons)
        self.create_timer(0.5, self.connect)
        self.get_logger().info(
            f'Joint control ready; Cartesian mode enabled={self.enable_kinematics}. '
            'Sending the original startup pose (1 second).')

        # 舵机初始化姿态
        bus_servo_control.set_servo_position(self.servos_pub, 1.0,
                                             ((6, 500), (5, 640), (4, 825), (3, 110), (2, 500), (1, 200)))
        


    def _declare_jog_parameters(self):
        """Joint settings support live updates; other jog settings need restart.

        Bounds below are conservative configuration limits, not collision or
        measured speed limits. Home/initialization actions are unchanged.
        """
        specs = (
            ('joint_step', 10, 1, 20, '单次关节点动步长，pulse；不是角度'),
            ('joint_duration', 0.05, 0.02, 30.0, '关节点动指令运动时间，秒'),
            ('cartesian_step', 0.005, 0.0001, 0.01, '单次笛卡尔平移步长，米'),
            ('cartesian_duration', 0.02, 0.02, 30.0, 'IK 结果的舵机指令运动时间，秒'),
            ('pitch_step', 1.0, 0.1, 2.0, '单次笛卡尔俯仰步长，度'),
        )
        values = {}
        for name, default, lower, upper, description in specs:
            descriptor = ParameterDescriptor(
                description=f'{description}；范围 [{lower}, {upper}]；'
                            + ('运行时可调' if name in JOINT_SETTINGS else '重启生效'),
                read_only=name not in JOINT_SETTINGS,
            )
            value = self.declare_parameter(name, default, descriptor=descriptor).value
            if (type(value) is not type(default) or not math.isfinite(value)
                    or not lower <= value <= upper):
                raise ValueError(
                    f'{name} must be a finite {type(default).__name__} '
                    f'in [{lower}, {upper}], got {value!r}')
            values[name] = value
        for name, value in values.items():
            setattr(self, name, value)
        self.get_logger().info(
            'Joystick jog settings (joint_step/joint_duration support live updates): '
            + ', '.join(f'{name}={value}' for name, value in values.items()))

        self.add_on_set_parameters_callback(self._validate_live_jog_parameters)
        self.add_post_set_parameters_callback(self._commit_live_jog_parameters)

    def _validate_live_jog_parameters(self, parameters):
        # Validation must not mutate cached settings: a later callback can reject.
        try:
            for parameter in parameters:
                if parameter.name in JOINT_SETTINGS:
                    validate_joint_setting(parameter.name, parameter.value)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _commit_live_jog_parameters(self, parameters):
        # ROS parameter services and input timers share the default mutually
        # exclusive callback group. No input callback can observe a half update.
        for parameter in parameters:
            if parameter.name in JOINT_SETTINGS:
                setattr(self, parameter.name, parameter.value)

    def get_button_state(self, button):
        if button in self.BUTTONS:
            return self.last_buttons[self.BUTTONS.index(button)]
        return 0

    def servo_states_callback(self, msg):
        positions = {item.id: item.position for item in msg.servo_state}
        if not all(servo_id in positions for servo_id in (6, 5, 4, 3, 2, 1)):
            self.current_servo_position = None
            return
        # The message ordering is not part of the ID-to-button mapping.
        self.current_servo_position = np.array(
            [positions[servo_id] for servo_id in (6, 5, 4, 3, 2, 1)])
    
    def run_async(self, func, *args):
        """ 在新线程执行 func """
        threading.Thread(target=func, args=args, daemon=True).start()

    def send_request(self, client, msg):
        if client is None or not client.service_is_ready():
            self.get_logger().warning('Kinematics service unavailable; no motion sent')
            return None
        future = client.call_async(msg)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=1.0):
            future.cancel()
            self.get_logger().warning('Kinematics request timed out; no motion sent')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'Kinematics request failed: {exc}')
            return None

    def relative_move_async(self, x, y, z, pitch):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request())
        if endpoint is None:
            return
        pose = endpoint.pose
        x += pose.position.x
        y += pose.position.y
        z += pose.position.z
        rpy = transform.qua2rpy(endpoint.pose.orientation)
        self.pitch = pitch + rpy[1]
        req = set_pose_target([x, y, z], self.pitch, pitch_range=[-50, 50], resolution=1)
        res = self.send_request(self.kinematics_client, req)
        if res is not None and res.pulse : # Reachable 可以达到
            servo_data = res.pulse  
          
            # # Drive the servos 驱动舵机
            bus_servo_control.set_servo_position(
                    self.servos_pub, self.cartesian_duration,
                    ((3, servo_data[3]), (4, servo_data[2]),
                     (5, servo_data[1]), (6, servo_data[0]))
                )


    def connect(self):
        if self.js is not None:
            if self.js.get_init():
                return
            self._disconnect_gamepad()
        pg.joystick.init()
        if pg.joystick.get_count() == 0:
            return
        candidate = None
        try:
            candidate = pg.joystick.Joystick(0)
            candidate.init()
            name = candidate.get_name().strip()
            layouts = {'SHANWAN Android Gamepad': BUTTONS[0],
                       'USB WirelessGamepad': BUTTONS[1]}
            if name not in layouts:
                self.get_logger().warning(f'Unsupported gamepad {name!r}; input ignored')
                candidate.quit()
                return
            self.BUTTONS = layouts[name]
            self.last_buttons = [0] * len(self.BUTTONS)
            self.last_axes = dict.fromkeys(AXES_MAP, 0.0)
            # Drop queued input from before connection; require a new action.
            pg.event.clear()
            self.js = candidate
            self.get_logger().info(
                f'Gamepad connected: {name!r}; buttons={candidate.get_numbuttons()}, '
                f'axes={candidate.get_numaxes()}, hats={candidate.get_numhats()}')
        except pg.error as exc:
            if candidate is not None:
                candidate.quit()
            self.js = None
            self.get_logger().error(f'Gamepad connection failed: {exc}')

    def _disconnect_gamepad(self):
        if self.js is not None:
            self.js.quit()
        self.js = None
        self.last_axes = dict.fromkeys(AXES_MAP, 0.0)
        self.last_buttons = [0] * len(self.BUTTONS)
        self.get_logger().warning('Gamepad disconnected; input state cleared')

    def axes_callback(self, axes):
        if axes['hat_x'] == 1:  # 上
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[1] = self.current_servo_position[1] + self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((5, self.current_servo_position[1]), ))          
            else:
                self.relative_move_async(self.cartesian_step, 0, 0, 0)
        elif axes['hat_x'] == -1:  # 下
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[1] = self.current_servo_position[1] - self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((5, self.current_servo_position[1]), ))          
            else:
                self.relative_move_async(-self.cartesian_step, 0, 0, 0)

        # 使用十字键的左右控制机器人旋转
        if axes['hat_y'] == 1:  # 右
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[0] = self.current_servo_position[0] - self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((6, self.current_servo_position[0]), ))          
            else:
                self.relative_move_async(0, -self.cartesian_step, 0, 0)
        elif axes['hat_y'] == -1:  # 左
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[0] = self.current_servo_position[0] + self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((6, self.current_servo_position[0]), ))          
            else:
                self.relative_move_async(0, self.cartesian_step, 0, 0)        
            
        # self.mecanum_pub.publish(twist)
   
    def handle_button_event(self, button_state, pressed):
        """
        处理按键事件：按下和释放
        """
        if not 0 <= button_state < len(self.BUTTONS):
            return
        self.last_buttons[button_state] = int(pressed)
        callback = "".join([self.BUTTONS[button_state], '_callback'])
        if hasattr(self, callback):
            try:
                getattr(self, callback)(pressed)
            except Exception as e:
                self.get_logger().error(str(e))
    
        
    def mode_callback(self, new_state):
        if new_state:
            if self.mode == 0 and (
                    not self.enable_kinematics
                    or not self.get_current_pose_client.service_is_ready()
                    or not self.kinematics_client.service_is_ready()):
                self.get_logger().warning(
                    'Cartesian mode unavailable; staying in joint mode. '
                    'It requires enable_kinematics:=true and working FK/IK services.')
                return
            self.mode = 1 if self.mode == 0 else 0
            self.get_logger().info(f"模式切换为: {self.mode}")
            if self.mode == 0:
                msg = BuzzerState()
                msg.freq = 1500
                msg.on_time = 0.05
                msg.off_time = 0.01
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
            else:
                msg = BuzzerState()
                msg.freq = 2500
                msg.on_time = 0.05
                msg.off_time = 0.01
                msg.repeat = 2
                self.buzzer_pub.publish(msg)
        

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[5] = self.current_servo_position[5] - self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((1, self.current_servo_position[5]), ))          
            else:
                self.relative_move_async(0, 0, self.cartesian_step, 0)

    def l2_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[5] = self.current_servo_position[5] + self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((1, self.current_servo_position[5]), ))          
            else:
                self.relative_move_async(0, 0, -self.cartesian_step, 0)

    def r1_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[3] = self.current_servo_position[3] + self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((3, self.current_servo_position[3]), ))          
            else:
                self.relative_move_async(0, 0, 0, -self.pitch_step)


    def r2_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return
            if self.mode == 0:     
                self.current_servo_position[3] = self.current_servo_position[3] - self.joint_step
                bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((3, self.current_servo_position[3]), ))          
            else:
                self.relative_move_async(0, 0, 0, self.pitch_step)

    def square_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return    
            self.current_servo_position[4] = self.current_servo_position[4] + self.joint_step
            bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((2, self.current_servo_position[4]), ))          


    def circle_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return    
            self.current_servo_position[4] = self.current_servo_position[4] - self.joint_step
            bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((2, self.current_servo_position[4]), ))          

    def cross_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return    
            self.current_servo_position[2] = self.current_servo_position[2] - self.joint_step
            bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((4, self.current_servo_position[2]), ))          


    def triangle_callback(self, new_state):
        if new_state:           
            if self.current_servo_position is None:
                self.get_logger().warn("Servo state not ready yet")
                return    
            self.current_servo_position[2] = self.current_servo_position[2] + self.joint_step
            bus_servo_control.set_servo_position(self.servos_pub, self.joint_duration, ((4, self.current_servo_position[2]), ))          



    def start_callback(self, new_state):
        if new_state:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
            self.run_async(actions.go_home, self.servos_pub, 1.0)  
    
    def update_buttons(self):
        # Return after one batch so servo-state and parameter callbacks can run.
        if self.js is None:
            pg.event.clear()
            return
        try:
            events = pg.event.get()
            # Discard the whole batch if the active device was disconnected.
            # get_init() checks local initialization only. Physical removal
            # is reported by JOYDEVICEREMOVED with this device's instance ID.
            # pygame.joystick.Joystick has no get_attached() API.
            if not self.js.get_init() or any(
                    event.type == pg.JOYDEVICEREMOVED
                    and event.instance_id == self.js.get_instance_id()
                    for event in events):
                self._disconnect_gamepad()
                return
            for event in events:
                if event.type == pg.QUIT:
                    rclpy.shutdown()
                    return
                if getattr(event, 'instance_id', None) != self.js.get_instance_id():
                    continue
                if event.type == pg.JOYAXISMOTION:
                    if 0 <= event.axis < 4:
                        self.last_axes[AXES_MAP[event.axis]] = event.value
                elif event.type == pg.JOYHATMOTION and event.hat == 0:
                    hat_y, hat_x = event.value
                    self.last_axes['hat_x'] = hat_x
                    self.last_axes['hat_y'] = hat_y
                    self.axes_callback(self.last_axes)
                elif event.type == pg.JOYBUTTONDOWN:
                    self.handle_button_event(event.button, True)
                elif event.type == pg.JOYBUTTONUP:
                    self.handle_button_event(event.button, False)
        except pg.error as exc:
            self.get_logger().error(f'Gamepad input failed: {exc}')
            self._disconnect_gamepad()


def main():
    node = JoystickController('joystick_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node() 

if __name__ == "__main__":
    main()