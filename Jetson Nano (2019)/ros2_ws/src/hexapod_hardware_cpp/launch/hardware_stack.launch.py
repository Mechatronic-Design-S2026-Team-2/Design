from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def _default_linkage():
    pkg_share = get_package_share_directory('hexapod_hardware_cpp')
    prefix = get_package_prefix('hexapod_hardware_cpp')
    ws_root = os.path.dirname(os.path.dirname(prefix))
    generated = os.path.join(ws_root, 'hexapod_lut_out', 'linkage_precomputed.yaml')
    if os.path.exists(generated):
        return generated
    return os.path.join(pkg_share, 'config', 'linkage_measured.yaml')


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_hardware_cpp')
    params = os.path.join(pkg_share, 'config', 'hardware_stack.yaml')
    linkage = _default_linkage()

    start_lcd_status = LaunchConfiguration('start_lcd_status')
    start_imu_udp_export = LaunchConfiguration('start_imu_udp_export')

    return LaunchDescription([
        DeclareLaunchArgument('start_lcd_status', default_value='true'),
        DeclareLaunchArgument('start_imu_udp_export', default_value='false'),
        DeclareLaunchArgument('lcd_i2c_device', default_value='/dev/i2c-1'),
        DeclareLaunchArgument('lcd_i2c_address', default_value='39'),
        DeclareLaunchArgument('lcd_operator_url', default_value='http://192.168.50.1:8080/'),
        DeclareLaunchArgument('lcd_page_period_sec', default_value='3.0'),
        DeclareLaunchArgument('lcd_refresh_period_ms', default_value='750'),
        DeclareLaunchArgument('lcd_static_url_only', default_value='true'),
        Node(package='hexapod_hardware_cpp', executable='wt901_imu_node', name='wt901_imu_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='imu_udp_export_node', name='imu_udp_export_node', parameters=[params], condition=IfCondition(start_imu_udp_export)),
        Node(package='hexapod_hardware_cpp', executable='motor_state_aggregator_node', name='motor_state_aggregator_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='phase_command_router_node', name='phase_command_router_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='body_state_estimator_node', name='body_state_estimator_node', parameters=[params, {'linkage_yaml': linkage}]),
        Node(package='hexapod_hardware_cpp', executable='weighted_odom_fusion_node', name='weighted_odom_fusion_node', parameters=[params]),
        Node(
            package='hexapod_hardware_cpp',
            executable='lcd1602_status_node',
            name='lcd1602_status_node',
            output='screen',
            condition=IfCondition(start_lcd_status),
            parameters=[params, {
                'enabled': True,
                'i2c_device': LaunchConfiguration('lcd_i2c_device'),
                'i2c_address': ParameterValue(LaunchConfiguration('lcd_i2c_address'), value_type=int),
                'operator_url': LaunchConfiguration('lcd_operator_url'),
                'page_period_sec': ParameterValue(LaunchConfiguration('lcd_page_period_sec'), value_type=float),
                'refresh_period_ms': ParameterValue(LaunchConfiguration('lcd_refresh_period_ms'), value_type=int),
                'static_url_only': ParameterValue(LaunchConfiguration('lcd_static_url_only'), value_type=bool),
                'status_json_topic': '/hexapod/operator/status_json',
                'battery_topic': '/hexapod/operator/battery_voltage',
                'selected_cmd_topic': '/hexapod/cmd_vel_selected',
            }],
        ),
    ])
