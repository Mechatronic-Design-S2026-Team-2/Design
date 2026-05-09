import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    start_tripod_controller = LaunchConfiguration('start_tripod_controller')
    start_hardware_stack = LaunchConfiguration('start_hardware_stack')
    start_orbslam_slamtoolbox = LaunchConfiguration('start_orbslam_slamtoolbox')
    start_operator_interface = LaunchConfiguration('start_operator_interface')
    start_scan_gate = LaunchConfiguration('start_scan_gate')
    start_merged_visualization = LaunchConfiguration('start_merged_visualization')
    nav2_start_delay_sec = LaunchConfiguration('nav2_start_delay_sec')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav_raw'),
                    ('cmd_vel_smoothed', 'cmd_vel_nav'),
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ]
    )

    hardware_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('hexapod_hardware_cpp'),
                'launch',
                'hardware_stack.launch.py',
            ])
        ),
        condition=IfCondition(start_hardware_stack),
        launch_arguments={
            'start_lcd_status': LaunchConfiguration('start_lcd_status'),
            'start_imu_udp_export': LaunchConfiguration('start_imu_udp_export'),
            'lcd_i2c_device': LaunchConfiguration('lcd_i2c_device'),
            'lcd_i2c_address': LaunchConfiguration('lcd_i2c_address'),
            'lcd_operator_url': LaunchConfiguration('operator_url'),
            'lcd_page_period_sec': LaunchConfiguration('lcd_page_period_sec'),
            'lcd_refresh_period_ms': LaunchConfiguration('lcd_refresh_period_ms'),
            'lcd_static_url_only': LaunchConfiguration('lcd_static_url_only'),
        }.items(),
    )

    merged_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('hexapod_hardware_cpp'),
                'launch',
                'merged_visualization.launch.py',
            ])
        ),
        condition=IfCondition(start_merged_visualization),
    )

    # Start the gait controller directly instead of including its launch file.
    # This avoids a class of launch-argument propagation failures where string
    # topic overrides can silently fall back to tripod_gait.yaml defaults. The
    # operator web UI publishes to /hexapod/cmd_vel_selected, so the controller
    # must subscribe there in the integrated Nav2 stack.
    tripod_config_file = PathJoinSubstitution([
        FindPackageShare('hexapod_tripod_gait'),
        'config',
        'tripod_gait.yaml',
    ])

    tripod_launch = Node(
        package='hexapod_tripod_gait',
        executable='tripod_gait_node',
        name='hexapod_tripod_gait',
        output='screen',
        condition=IfCondition(start_tripod_controller),
        parameters=[
            tripod_config_file,
            {
                'cmd_vel_topic': '/hexapod/cmd_vel_selected',
                'cmd_vel_stamped_topic': '',
                'command_output_mode': 'phase',
                'phase_command_topic': 'hexapod/phase_cmd',
                'use_motor_state_feedback': True,
                'use_compact_odometry_feedback': False,
                'motor_state_require_comm_ok': False,
                'motor_state_require_fresh': False,
                'publish_rate_hz': 10.0,
                'linear_rpm_per_mps': LaunchConfiguration('tripod_linear_rpm_per_mps'),
                'yaw_rpm_per_radps': LaunchConfiguration('tripod_yaw_rpm_per_radps'),
                'max_motor_rpm': LaunchConfiguration('tripod_max_motor_rpm'),
                'phase_lock_enabled': True,
                'phase_kp_rpm_per_rad': LaunchConfiguration('tripod_phase_kp_rpm_per_rad'),
                'max_phase_correction_rpm': LaunchConfiguration('tripod_max_phase_correction_rpm'),
                'imu_feedback_enabled': True,
                'euler_feedback_enabled': True,
                'yaw_rate_feedback_enabled': LaunchConfiguration('tripod_yaw_rate_feedback_enabled'),
                'yaw_rate_feedback_kp': LaunchConfiguration('tripod_yaw_rate_feedback_kp'),
                'max_yaw_rate_correction_radps': LaunchConfiguration('tripod_max_yaw_rate_correction_radps'),
                'linear_accel_feedback_enabled': False,
                'tilt_safety_enabled': True,
                'publish_unchanged_commands': True,
                'latch_last_teleop_command': True,
            },
        ],
    )

    operator_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('hexapod_operator_interface'),
                'launch',
                'operator_interface.launch.py',
            ])
        ),
        condition=IfCondition(start_operator_interface),
        launch_arguments={
            'selected_cmd_vel_topic': '/hexapod/cmd_vel_selected',
            'nav_cmd_vel_topic': '/cmd_vel_nav',
            'default_mode': LaunchConfiguration('operator_default_mode'),
            'latch_teleop_commands': LaunchConfiguration('operator_latch_teleop_commands'),
            'http_port': LaunchConfiguration('operator_http_port'),
            'bind_address': LaunchConfiguration('operator_bind_address'),
            'keyboard_linear_mps': LaunchConfiguration('operator_keyboard_linear_mps'),
            'keyboard_angular_radps': LaunchConfiguration('operator_keyboard_angular_radps'),
            'max_linear_mps': LaunchConfiguration('operator_max_linear_mps'),
            'max_angular_radps': LaunchConfiguration('operator_max_angular_radps'),
            'waypoint_max_linear_mps': LaunchConfiguration('operator_waypoint_max_linear_mps'),
            'waypoint_max_angular_radps': LaunchConfiguration('operator_waypoint_max_angular_radps'),
            'map_storage_dir': LaunchConfiguration('operator_map_storage_dir'),
            'host_control_base_url': LaunchConfiguration('operator_host_control_base_url'),
            'mapping_updates_enabled_default': LaunchConfiguration('operator_mapping_updates_enabled_default'),
            'mapping_enable_topic': LaunchConfiguration('operator_mapping_enable_topic'),
        }.items(),
    )


    scan_gate_node = Node(
        package='hexapod_operator_interface',
        executable='scan_gate_node',
        name='hexapod_scan_gate',
        output='screen',
        condition=IfCondition(start_scan_gate),
        parameters=[{
            'scan_in_topic': LaunchConfiguration('scan_gate_input_topic'),
            'mapping_scan_topic': LaunchConfiguration('scan_gate_mapping_topic'),
            'nav_scan_topic': LaunchConfiguration('scan_gate_nav_topic'),
            'mapping_enable_topic': LaunchConfiguration('operator_mapping_enable_topic'),
            'mapping_enabled_default': ParameterValue(LaunchConfiguration('operator_mapping_updates_enabled_default'), value_type=bool),
            'mapping_max_rate_hz': ParameterValue(LaunchConfiguration('scan_gate_mapping_max_rate_hz'), value_type=float),
            'nav_max_rate_hz': ParameterValue(LaunchConfiguration('scan_gate_nav_max_rate_hz'), value_type=float),
            'restamp_scans': ParameterValue(LaunchConfiguration('scan_gate_restamp_scans'), value_type=bool),
        }],
    )


    orbslam_slamtoolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('hexapod_orbslam_udp_bridge'),
                'launch',
                'orbslam_slam_toolbox.launch.py',
            ])
        ),
        condition=IfCondition(start_orbslam_slamtoolbox),
        launch_arguments={
            'udp_port': LaunchConfiguration('orbslam_udp_port'),
            'udp_bind_ip': LaunchConfiguration('orbslam_udp_bind_ip'),
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'odom_topic': '/hexapod/orbslam_odom',
            'publish_odom': 'true',
            'publish_tf': 'false',
            'publish_scan': 'true',
            'scan_topic': LaunchConfiguration('scan_gate_input_topic'),
            'publish_map': 'true',
            'publish_scan_static_tf': 'true',
            'scan_frame': 'orbslam_scan_frame',
            'map_frame': LaunchConfiguration('orbslam_map_frame'),
            'convert_orbslam_to_ros_axes': LaunchConfiguration('orbslam_convert_orbslam_to_ros_axes'),
            'convert_scan_optical_to_ros': LaunchConfiguration('orbslam_convert_scan_optical_to_ros'),
            'use_packet_stamp': LaunchConfiguration('orbslam_use_packet_stamp'),
            'stamp_scan_with_latest_pose': LaunchConfiguration('orbslam_stamp_scan_with_latest_pose'),
            'position_scale': LaunchConfiguration('orbslam_position_scale'),
            'scan_range_scale': LaunchConfiguration('orbslam_scan_range_scale'),
            'scan_publish_decimation': LaunchConfiguration('orbslam_scan_publish_decimation'),
            'map_points_apply_static_offset': LaunchConfiguration('orbslam_map_points_apply_static_offset'),
            'map_points_offset_x': LaunchConfiguration('orbslam_map_points_offset_x'),
            'map_points_offset_y': '0.0',
            'map_points_offset_z': '0.0',
            'camopt_to_base_x': '0.0',
            'camopt_to_base_y': '0.0',
            'camopt_to_base_z': '-0.2794',
            'scan_x': LaunchConfiguration('orbslam_scan_x'),
            'scan_y': LaunchConfiguration('orbslam_scan_y'),
            'scan_z': LaunchConfiguration('orbslam_scan_z'),
            'scan_roll': '0.0',
            'scan_pitch': '0.0',
            'scan_yaw': '0.0',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'hexapod_nav2_params.yaml'),
            description='Full path to the Nav2 parameter YAML file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'start_tripod_controller',
            default_value='true',
            description='Start the hexapod_tripod_gait controller in Nav2-safe mode.',
        ),
        DeclareLaunchArgument(
            'start_hardware_stack',
            default_value='true',
            description='Start hexapod_hardware_cpp aggregation, phase routing, body odom, and odom fusion nodes.',
        ),
        DeclareLaunchArgument(
            'start_orbslam_slamtoolbox',
            default_value='false',
            description='Also start the patched ORB-SLAM UDP bridge plus slam_toolbox launch. Keep false if already running.',
        ),
        DeclareLaunchArgument('start_scan_gate', default_value='true', description='Start scan gate that separates mapping scans from Nav2 obstacle scans.'),
        DeclareLaunchArgument(
            'start_operator_interface',
            default_value='true',
            description='Start browser HTTP operator UI and cmd_vel arbiter.',
        ),
        DeclareLaunchArgument('operator_http_port', default_value='8080'),
        DeclareLaunchArgument('operator_bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('operator_url', default_value='http://192.168.50.1:8080/'),
        DeclareLaunchArgument('start_lcd_status', default_value='true'),
        DeclareLaunchArgument('start_imu_udp_export', default_value='false'),
        DeclareLaunchArgument('lcd_i2c_device', default_value='/dev/i2c-1'),
        DeclareLaunchArgument('lcd_i2c_address', default_value='39'),
        DeclareLaunchArgument('lcd_page_period_sec', default_value='3.0'),
        DeclareLaunchArgument('lcd_refresh_period_ms', default_value='750'),
        DeclareLaunchArgument('lcd_static_url_only', default_value='true'),
        DeclareLaunchArgument('operator_default_mode', default_value='estop'),
        DeclareLaunchArgument('operator_latch_teleop_commands', default_value='true'),
        DeclareLaunchArgument('operator_keyboard_linear_mps', default_value='0.50'),
        DeclareLaunchArgument('operator_keyboard_angular_radps', default_value='0.50'),
        DeclareLaunchArgument('operator_max_linear_mps', default_value='3.00'),
        DeclareLaunchArgument('operator_max_angular_radps', default_value='3.00'),
        DeclareLaunchArgument('operator_waypoint_max_linear_mps', default_value='0.50'),
        DeclareLaunchArgument('operator_waypoint_max_angular_radps', default_value='0.50'),
        DeclareLaunchArgument('operator_map_storage_dir', default_value='/ros2_ws/maps'),
        DeclareLaunchArgument('operator_host_control_base_url', default_value='http://127.0.0.1:18080'),
        DeclareLaunchArgument('operator_mapping_updates_enabled_default', default_value='true'),
        DeclareLaunchArgument('operator_mapping_enable_topic', default_value='/hexapod/operator/map_updates_enabled'),
        DeclareLaunchArgument('scan_gate_input_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('scan_gate_mapping_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_gate_nav_topic', default_value='/scan_nav'),
        DeclareLaunchArgument('scan_gate_mapping_max_rate_hz', default_value='3.0'),
        DeclareLaunchArgument('scan_gate_nav_max_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('scan_gate_restamp_scans', default_value='true'),
        DeclareLaunchArgument('start_merged_visualization', default_value='false'),
        DeclareLaunchArgument('nav2_start_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('orbslam_udp_port', default_value='5005'),
        DeclareLaunchArgument('orbslam_udp_bind_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('orbslam_convert_orbslam_to_ros_axes', default_value='false'),
        DeclareLaunchArgument('orbslam_use_packet_stamp', default_value='false'),
        DeclareLaunchArgument('orbslam_convert_scan_optical_to_ros', default_value='true'),
        DeclareLaunchArgument('orbslam_stamp_scan_with_latest_pose', default_value='false'),
        DeclareLaunchArgument('orbslam_position_scale', default_value='1.214'),
        DeclareLaunchArgument('orbslam_scan_range_scale', default_value='1.214'),
        DeclareLaunchArgument('orbslam_scan_publish_decimation', default_value='3'),
        DeclareLaunchArgument('orbslam_map_frame', default_value='map'),
        DeclareLaunchArgument('orbslam_map_points_apply_static_offset', default_value='true'),
        DeclareLaunchArgument('orbslam_map_points_offset_x', default_value='0.2794'),
        DeclareLaunchArgument('orbslam_scan_x', default_value='0.2794'),
        DeclareLaunchArgument('orbslam_scan_y', default_value='0.0'),
        DeclareLaunchArgument('orbslam_scan_z', default_value='0.0'),
        DeclareLaunchArgument('tripod_linear_rpm_per_mps', default_value='500.0'),
        DeclareLaunchArgument('tripod_yaw_rpm_per_radps', default_value='250.0'),
        DeclareLaunchArgument('tripod_max_motor_rpm', default_value='500.0'),
        DeclareLaunchArgument('tripod_phase_kp_rpm_per_rad', default_value='35.0'),
        DeclareLaunchArgument('tripod_max_phase_correction_rpm', default_value='100.0'),
        DeclareLaunchArgument('tripod_yaw_rate_feedback_enabled', default_value='true'),
        DeclareLaunchArgument('tripod_yaw_rate_feedback_kp', default_value='0.10'),
        DeclareLaunchArgument('tripod_max_yaw_rate_correction_radps', default_value='0.10'),
        orbslam_slamtoolbox_launch,
        hardware_stack_launch,
        merged_visualization_launch,
        operator_interface_launch,
        scan_gate_node,
        tripod_launch,
        TimerAction(period=nav2_start_delay_sec, actions=[nav2_nodes]),
    ])
