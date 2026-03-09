from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    default_params = PathJoinSubstitution([FindPackageShare('hexapod_control_gui'), 'config', 'gui_defaults.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='hexapod_control_gui',
            executable='hexapod_control_gui',
            name='hexapod_control_gui',
            output='screen',
            parameters=[params_file],
        ),
    ])
