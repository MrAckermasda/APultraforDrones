import threading, os, time
import subprocess
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board()
#board.enable_reception()

def getServoPulse(servo_id):
    ret = board.bus_servo_read_position(servo_id)
    if ret is not None:
        return ret[0]

def getServoDeviation(servo_id):
    ret = board.bus_servo_read_offset(servo_id)
    if ret is not None:
        return ret[0]

def setServoPulse(servo_id, pulse, use_time):
    board.bus_servo_set_position(use_time/1000.0, ((servo_id, pulse),))
    
def setBusServoPulse(servo_id, pulse, use_time):
    board.bus_servo_set_position(use_time/1000.0, ((servo_id, pulse),))

def setServoDeviation(servo_id ,dev):
    board.bus_servo_set_offset(servo_id, dev)
    
def saveServoDeviation(servo_id):
    board.bus_servo_save_offset(servo_id)

def unloadServo(servo_id):
    board.bus_servo_enable_torque(servo_id, True)

import action_group_controller as controller
controller.set_servos = setServoPulse

def runActionGroup(num):
    threading.Thread(target=controller.runAction, args=(num, )).start()    
  

def stopActionGroup():    
    controller.stop_action_group()  

def enable_reception(enable=True):
    if enable:
        board.enable_reception(not enable)
        time.sleep(1)
        threading.Thread(target=subprocess.run, args=([
            'ros2', 'topic', 'pub', '--once', '/ros_robot_controller/enable_reception',
            'std_msgs/msg/Bool', 'data: true',
        ],), kwargs={'check': False}, daemon=True).start()
        time.sleep(1)
    else:
        threading.Thread(target=subprocess.run, args=([
            'ros2', 'topic', 'pub', '--once', '/ros_robot_controller/enable_reception',
            'std_msgs/msg/Bool', 'data: false',
        ],), kwargs={'check': False}, daemon=True).start()
        time.sleep(3)
        board.enable_reception(not enable)
        time.sleep(1)


