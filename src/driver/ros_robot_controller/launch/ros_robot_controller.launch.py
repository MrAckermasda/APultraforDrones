from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=EnvironmentVariable(
            'ARMPI_SERIAL_PORT', default_value='/dev/rrc')),
        DeclareLaunchArgument('baudrate', default_value=EnvironmentVariable(
            'ARMPI_BAUDRATE', default_value='1000000')),
        Node(
            package='ros_robot_controller', executable='ros_robot_controller',
            output='screen', parameters=[{
                'serial_port': ParameterValue(LaunchConfiguration('serial_port'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            }],
        ),
    ])
