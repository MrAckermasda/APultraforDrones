import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # noqa: E402
from launch import LaunchDescription, LaunchService  # noqa: E402
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    peripherals_package_path = get_package_share_directory('peripherals')
    camera_nodes = Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name='usb_cam',
            parameters=[os.path.join(peripherals_package_path, 'config', 'usb_cam_param.yaml'),
                        {"pixel_format": "yuyv2rgb", "video_device": ParameterValue(
                            LaunchConfiguration('video_device'), value_type=str)},
                        ],
            remappings = [
                ('image_raw/compressed', '/depth_cam/rgb/image_compressed'),
                ('image_raw/compressedDepth', '/depth_cam/rgb/compressedDepth'),
                ('image_raw/theora', '/depth_cam/rgb/image_raw/theora'),
                ('camera_info', '/depth_cam/rgb/camera_info'),
            ]
        )
    image_proc_node = Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_node',
            remappings=[
                        ('image', 'image_raw'),
                        ('image_raw/compressed', '/depth_cam/rgb/image_compressed'),
                        ('image_raw/compressedDepth', '/depth_cam/rgb/compressedDepth'),
                        ('image_raw/theora', '/depth_cam/rgb/image_raw/theora'),
                        ('image_rect', '/depth_cam/rgb/image_raw'),
                        ('camera_info', '/depth_cam/rgb/camera_info'),
                        ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'error']  
        )

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value=EnvironmentVariable(
            'ARMPI_VIDEO_DEVICE', default_value='/dev/video0')),
        camera_nodes, image_proc_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
