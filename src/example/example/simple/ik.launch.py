from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    

    sdk_package_path = get_package_share_directory('sdk')


    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(sdk_package_path, 'launch/armpi_ultra.launch.py')
        ]),
    )


    ik_node = Node(
        package='example',
        executable='ik',
        output='screen',
            )

    return LaunchDescription([
        sdk_launch,
        ik_node
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
