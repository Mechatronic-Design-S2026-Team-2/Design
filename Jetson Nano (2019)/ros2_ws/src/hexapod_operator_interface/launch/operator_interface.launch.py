from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hexapod_operator_interface'),
                'config',
                'operator_interface.yaml',
            ]),
        ),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('selected_cmd_vel_topic', default_value='/hexapod/cmd_vel_selected'),
        DeclareLaunchArgument('nav_cmd_vel_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('default_mode', default_value='estop'),
        DeclareLaunchArgument('latch_teleop_commands', default_value='true'),
        DeclareLaunchArgument('keyboard_linear_mps', default_value='0.50'),
        DeclareLaunchArgument('keyboard_angular_radps', default_value='0.50'),
        DeclareLaunchArgument('max_linear_mps', default_value='3.00'),
        DeclareLaunchArgument('max_angular_radps', default_value='3.00'),
        DeclareLaunchArgument('waypoint_max_linear_mps', default_value='0.50'),
        DeclareLaunchArgument('waypoint_max_angular_radps', default_value='0.50'),
        DeclareLaunchArgument('map_storage_dir', default_value='/ros2_ws/maps'),
        DeclareLaunchArgument('host_control_base_url', default_value='http://127.0.0.1:18080'),
        DeclareLaunchArgument('mapping_updates_enabled_default', default_value='true'),
        DeclareLaunchArgument('mapping_enable_topic', default_value='/hexapod/operator/map_updates_enabled'),
        Node(
            package='hexapod_operator_interface',
            executable='operator_interface_node',
            name='hexapod_operator_interface',
            output='screen',
            parameters=[
                config_file,
                {
                    'http_port': ParameterValue(LaunchConfiguration('http_port'), value_type=int),
                    'bind_address': LaunchConfiguration('bind_address'),
                    'selected_cmd_vel_topic': LaunchConfiguration('selected_cmd_vel_topic'),
                    'nav_cmd_vel_topic': LaunchConfiguration('nav_cmd_vel_topic'),
                    'default_mode': LaunchConfiguration('default_mode'),
                    'latch_teleop_commands': ParameterValue(LaunchConfiguration('latch_teleop_commands'), value_type=bool),
                    'keyboard_linear_mps': ParameterValue(LaunchConfiguration('keyboard_linear_mps'), value_type=float),
                    'keyboard_angular_radps': ParameterValue(LaunchConfiguration('keyboard_angular_radps'), value_type=float),
                    'max_linear_mps': ParameterValue(LaunchConfiguration('max_linear_mps'), value_type=float),
                    'max_angular_radps': ParameterValue(LaunchConfiguration('max_angular_radps'), value_type=float),
                    'waypoint_max_linear_mps': ParameterValue(LaunchConfiguration('waypoint_max_linear_mps'), value_type=float),
                    'waypoint_max_angular_radps': ParameterValue(LaunchConfiguration('waypoint_max_angular_radps'), value_type=float),
                    'map_storage_dir': LaunchConfiguration('map_storage_dir'),
                    'host_control_base_url': LaunchConfiguration('host_control_base_url'),
                    'mapping_updates_enabled_default': ParameterValue(LaunchConfiguration('mapping_updates_enabled_default'), value_type=bool),
                    'mapping_enable_topic': LaunchConfiguration('mapping_enable_topic'),
                },
            ],
        ),
    ])
