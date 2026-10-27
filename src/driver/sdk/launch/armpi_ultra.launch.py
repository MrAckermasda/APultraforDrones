import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction

def launch_setup(context):
    chassis_type = os.environ.get('CHASSIS_TYPE', 'None')
    

    ros_robot_controller_package_path = get_package_share_directory('ros_robot_controller')
    servo_controller_package_path = get_package_share_directory('servo_controller')
    kinematics_package_path = get_package_share_directory('kinematics')

    chassis_launches = []
    if chassis_type in ('Mecanum', 'Tank'):
        chassis_package_path = get_package_share_directory('controller')
        chassis_launches.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(chassis_package_path, 'launch/controller.launch.py')),
        ))

    ros_robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_robot_controller_package_path, 'launch/ros_robot_controller.launch.py')),
    )

    servo_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(servo_controller_package_path, 'launch/servo_controller.launch.py')),
        launch_arguments={'enable_grasp': LaunchConfiguration('enable_kinematics')}.items(),
    )

    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kinematics_package_path, 'launch/kinematics_node.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_kinematics')),
    )

    check_servo_connection_node = Node(
        package='sdk',
        executable='check_servo_connection',
    )
    return [
        ros_robot_controller_launch,
        servo_controller_launch,
        kinematics_launch,
        #check_servo_connection_node,
    ] + chassis_launches

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_kinematics', default_value='true'),
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
