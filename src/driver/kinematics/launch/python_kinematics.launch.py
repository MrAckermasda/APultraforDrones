"""Deploy the Python solver without starting a controller or moving any joint."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('kinematics'),
                                  'config', 'python_kinematics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('node_name', default_value='kinematics_python'),
        DeclareLaunchArgument('enable_pose_targets', default_value='false', choices=['true', 'false']),
        Node(package='kinematics', executable='python_kinematics',
             name=LaunchConfiguration('node_name'), output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'enable_pose_targets': ParameterValue(LaunchConfiguration('enable_pose_targets'), value_type=bool),
             }]),
    ])
