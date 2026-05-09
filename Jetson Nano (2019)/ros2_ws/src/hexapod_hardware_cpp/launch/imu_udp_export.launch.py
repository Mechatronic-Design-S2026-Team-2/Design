from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_hardware_cpp')
    params = os.path.join(pkg_share, 'config', 'hardware_stack.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('target_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('target_port', default_value='5015'),
        DeclareLaunchArgument('input_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('min_publish_period_s', default_value='0.0'),
        Node(
            package='hexapod_hardware_cpp',
            executable='imu_udp_export_node',
            name='imu_udp_export_node',
            parameters=[
                params,
                {
                    'target_ip': LaunchConfiguration('target_ip'),
                    'target_port': ParameterValue(LaunchConfiguration('target_port'), value_type=int),
                    'input_topic': LaunchConfiguration('input_topic'),
                    'min_publish_period_s': ParameterValue(LaunchConfiguration('min_publish_period_s'), value_type=float),
                },
            ],
        ),
    ])
