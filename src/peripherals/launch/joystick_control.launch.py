from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Actual arm jog controls. The legacy max_linear/max_angular options below
    # are retained for compatibility, but are not consumed by this arm node.
    jog_specs = (
        ('joint_step', '10', int, 'Joint increment (pulse), 1..20'),
        ('joint_duration', '0.05', float, 'Joint move duration (s), 0.02..30'),
        ('cartesian_step', '0.005', float, 'Cartesian increment (m), 0.0001..0.01'),
        ('cartesian_duration', '0.02', float, 'IK servo move duration (s), 0.02..30'),
        ('pitch_step', '1.0', float, 'Cartesian pitch increment (degrees), 0.1..2'),
    )
    jog_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in jog_specs
    ]
    jog_parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in jog_specs
    }
    max_linear = LaunchConfiguration('max_linear', default='0.5')
    max_angular = LaunchConfiguration('max_angular', default='2.0')
    remap_cmd_vel = LaunchConfiguration('remap_cmd_vel', default='controller/cmd_vel')

    max_linear_arg = DeclareLaunchArgument(
        'max_linear', default_value=max_linear)
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value=max_angular)
    remap_cmd_vel_arg = DeclareLaunchArgument(
        'remap_cmd_vel', default_value=remap_cmd_vel)

    kinematics_arg = DeclareLaunchArgument(
        'enable_kinematics', default_value='false',
        description='Enable optional Cartesian mode; requires compatible FK/IK services')

    joystick_control_node = Node(
        package='peripherals',
        executable='joystick_control',
        name='joystick_control',
        output='screen',
        parameters=[
            {'max_linear': max_linear,
             'max_angular': max_angular,
             'disable_servo_control': True},
            {'enable_kinematics': ParameterValue(LaunchConfiguration('enable_kinematics'), value_type=bool)},
            jog_parameters,
        ],
        remappings=[('controller/cmd_vel', remap_cmd_vel)]
    )

    return LaunchDescription([
        *jog_arguments,
        max_linear_arg,
        max_angular_arg,
        remap_cmd_vel_arg,
        kinematics_arg,
        joystick_control_node
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


