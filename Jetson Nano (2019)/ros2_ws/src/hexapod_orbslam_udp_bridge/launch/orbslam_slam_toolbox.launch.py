from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_launch = PathJoinSubstitution([
        FindPackageShare('hexapod_orbslam_udp_bridge'),
        'launch',
        'udp_bridge.launch.py',
    ])

    default_slam_params = PathJoinSubstitution([
        FindPackageShare('hexapod_orbslam_udp_bridge'),
        'config',
        'slam_toolbox_orbslam_scan.yaml',
    ])

    slam_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    ])

    return LaunchDescription([
        SetEnvironmentVariable('GLOG_minloglevel', '1'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('start_slam_toolbox', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),

        DeclareLaunchArgument('udp_port', default_value='5005'),
        DeclareLaunchArgument('udp_bind_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('odom_topic', default_value='/hexapod/orbslam_odom'),
        DeclareLaunchArgument('publish_odom', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('publish_scan', default_value='true'),
        DeclareLaunchArgument('publish_map', default_value='true'),
        DeclareLaunchArgument('publish_scan_static_tf', default_value='true'),
        DeclareLaunchArgument('scan_frame', default_value='orbslam_scan_frame'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('convert_orbslam_to_ros_axes', default_value='false'),
        DeclareLaunchArgument('convert_scan_optical_to_ros', default_value='true'),
        DeclareLaunchArgument('use_packet_stamp', default_value='false'),
        DeclareLaunchArgument('stamp_scan_with_latest_pose', default_value='true'),
        DeclareLaunchArgument('position_scale', default_value='1.382'),
        DeclareLaunchArgument('scan_range_scale', default_value='1.382'),
        DeclareLaunchArgument('scan_publish_decimation', default_value='3'),
        DeclareLaunchArgument('map_points_apply_static_offset', default_value='true'),
        DeclareLaunchArgument('map_points_offset_x', default_value='0.2794'),
        DeclareLaunchArgument('map_points_offset_y', default_value='0.0'),
        DeclareLaunchArgument('map_points_offset_z', default_value='0.0'),

        DeclareLaunchArgument('camopt_to_base_x', default_value='0.0'),
        DeclareLaunchArgument('camopt_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('camopt_to_base_z', default_value='-0.2794'),

        DeclareLaunchArgument('scan_x', default_value='0.2794'),
        DeclareLaunchArgument('scan_y', default_value='0.0'),
        DeclareLaunchArgument('scan_z', default_value='0.0'),
        DeclareLaunchArgument('scan_roll', default_value='0.0'),
        DeclareLaunchArgument('scan_pitch', default_value='0.0'),
        DeclareLaunchArgument('scan_yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            condition=IfCondition(LaunchConfiguration('start_bridge')),
            launch_arguments={
                'udp_port': LaunchConfiguration('udp_port'),
                'udp_bind_ip': LaunchConfiguration('udp_bind_ip'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_odom': LaunchConfiguration('publish_odom'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_scan': LaunchConfiguration('publish_scan'),
                'publish_map': LaunchConfiguration('publish_map'),
                'publish_scan_static_tf': LaunchConfiguration('publish_scan_static_tf'),
                'scan_frame': LaunchConfiguration('scan_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'convert_orbslam_to_ros_axes': LaunchConfiguration('convert_orbslam_to_ros_axes'),
                'convert_scan_optical_to_ros': LaunchConfiguration('convert_scan_optical_to_ros'),
                'use_packet_stamp': LaunchConfiguration('use_packet_stamp'),
                'stamp_scan_with_latest_pose': LaunchConfiguration('stamp_scan_with_latest_pose'),
                'position_scale': LaunchConfiguration('position_scale'),
                'scan_range_scale': LaunchConfiguration('scan_range_scale'),
                'scan_publish_decimation': LaunchConfiguration('scan_publish_decimation'),
                'map_points_apply_static_offset': LaunchConfiguration('map_points_apply_static_offset'),
                'map_points_offset_x': LaunchConfiguration('map_points_offset_x'),
                'map_points_offset_y': LaunchConfiguration('map_points_offset_y'),
                'map_points_offset_z': LaunchConfiguration('map_points_offset_z'),
                'camopt_to_base_x': LaunchConfiguration('camopt_to_base_x'),
                'camopt_to_base_y': LaunchConfiguration('camopt_to_base_y'),
                'camopt_to_base_z': LaunchConfiguration('camopt_to_base_z'),
                'scan_x': LaunchConfiguration('scan_x'),
                'scan_y': LaunchConfiguration('scan_y'),
                'scan_z': LaunchConfiguration('scan_z'),
                'scan_roll': LaunchConfiguration('scan_roll'),
                'scan_pitch': LaunchConfiguration('scan_pitch'),
                'scan_yaw': LaunchConfiguration('scan_yaw'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(LaunchConfiguration('start_slam_toolbox')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
