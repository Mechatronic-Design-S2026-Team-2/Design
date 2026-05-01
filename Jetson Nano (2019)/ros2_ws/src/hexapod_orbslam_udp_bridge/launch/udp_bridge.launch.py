from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='5005'),
        DeclareLaunchArgument('udp_bind_ip', default_value='0.0.0.0'),

        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic', default_value='/hexapod/orbslam_odom'),

        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_frame', default_value='orbslam_scan_frame'),
        DeclareLaunchArgument('map_topic', default_value='/orbslam/map_points'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        DeclareLaunchArgument('publish_odom', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('publish_scan', default_value='true'),
        DeclareLaunchArgument('publish_map', default_value='true'),
        DeclareLaunchArgument('use_packet_stamp', default_value='false'),
        DeclareLaunchArgument('publish_when_lost', default_value='false'),
        DeclareLaunchArgument('publish_scan_when_lost', default_value='false'),

        DeclareLaunchArgument('apply_optical_to_base', default_value='true'),
        DeclareLaunchArgument('convert_orbslam_to_ros_axes', default_value='false'),
        DeclareLaunchArgument('position_scale', default_value='1.382'),
        DeclareLaunchArgument('scan_range_scale', default_value='1.382'),

        # Camera optical -> base_link translation, expressed in camera optical axes.
        # For a D415 11 in forward of base_link with no height/lateral offset:
        # [optical_x_right, optical_y_down, optical_z_forward] = [0, 0, -0.2794].
        DeclareLaunchArgument('camopt_to_base_x', default_value='0.0'),
        DeclareLaunchArgument('camopt_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('camopt_to_base_z', default_value='-0.2794'),

        # Sparse ORB map points are in the first camera/world origin. Add the
        # base->camera static offset so they overlay the base-origin occupancy map.
        DeclareLaunchArgument('map_points_apply_static_offset', default_value='true'),
        DeclareLaunchArgument('map_points_offset_x', default_value='0.2794'),
        DeclareLaunchArgument('map_points_offset_y', default_value='0.0'),
        DeclareLaunchArgument('map_points_offset_z', default_value='0.0'),

        # base_link -> scan_frame static transform, expressed in base_link axes.
        DeclareLaunchArgument('publish_scan_static_tf', default_value='true'),
        DeclareLaunchArgument('scan_x', default_value='0.2794'),
        DeclareLaunchArgument('scan_y', default_value='0.0'),
        DeclareLaunchArgument('scan_z', default_value='0.0'),
        DeclareLaunchArgument('scan_roll', default_value='0.0'),
        DeclareLaunchArgument('scan_pitch', default_value='0.0'),
        DeclareLaunchArgument('scan_yaw', default_value='0.0'),

        DeclareLaunchArgument('stamp_scan_with_latest_pose', default_value='true'),
        DeclareLaunchArgument('convert_scan_optical_to_ros', default_value='true'),
        DeclareLaunchArgument('invalid_scan_range_is_infinity', default_value='true'),
        DeclareLaunchArgument('scan_time_sec', default_value='0.033333333'),
        DeclareLaunchArgument('scan_publish_decimation', default_value='3'),

        Node(
            package='hexapod_orbslam_udp_bridge',
            executable='udp_bridge',
            name='orbslam_udp_bridge',
            output='screen',
            parameters=[{
                'udp_port': ParameterValue(LaunchConfiguration('udp_port'), value_type=int),
                'udp_bind_ip': LaunchConfiguration('udp_bind_ip'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'scan_frame': LaunchConfiguration('scan_frame'),
                'map_topic': LaunchConfiguration('map_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'map_points_apply_static_offset': ParameterValue(LaunchConfiguration('map_points_apply_static_offset'), value_type=bool),
                'map_points_offset_x': ParameterValue(LaunchConfiguration('map_points_offset_x'), value_type=float),
                'map_points_offset_y': ParameterValue(LaunchConfiguration('map_points_offset_y'), value_type=float),
                'map_points_offset_z': ParameterValue(LaunchConfiguration('map_points_offset_z'), value_type=float),

                'publish_odom': ParameterValue(LaunchConfiguration('publish_odom'), value_type=bool),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'publish_scan': ParameterValue(LaunchConfiguration('publish_scan'), value_type=bool),
                'publish_map': ParameterValue(LaunchConfiguration('publish_map'), value_type=bool),
                'use_packet_stamp': ParameterValue(LaunchConfiguration('use_packet_stamp'), value_type=bool),
                'publish_when_lost': ParameterValue(LaunchConfiguration('publish_when_lost'), value_type=bool),
                'publish_scan_when_lost': ParameterValue(LaunchConfiguration('publish_scan_when_lost'), value_type=bool),

                'apply_optical_to_base': ParameterValue(LaunchConfiguration('apply_optical_to_base'), value_type=bool),
                'convert_orbslam_to_ros_axes': ParameterValue(LaunchConfiguration('convert_orbslam_to_ros_axes'), value_type=bool),
                'position_scale': ParameterValue(LaunchConfiguration('position_scale'), value_type=float),
                'scan_range_scale': ParameterValue(LaunchConfiguration('scan_range_scale'), value_type=float),
                'camopt_to_base_x': ParameterValue(LaunchConfiguration('camopt_to_base_x'), value_type=float),
                'camopt_to_base_y': ParameterValue(LaunchConfiguration('camopt_to_base_y'), value_type=float),
                'camopt_to_base_z': ParameterValue(LaunchConfiguration('camopt_to_base_z'), value_type=float),

                'stamp_scan_with_latest_pose': ParameterValue(LaunchConfiguration('stamp_scan_with_latest_pose'), value_type=bool),
                'convert_scan_optical_to_ros': ParameterValue(LaunchConfiguration('convert_scan_optical_to_ros'), value_type=bool),
                'invalid_scan_range_is_infinity': ParameterValue(LaunchConfiguration('invalid_scan_range_is_infinity'), value_type=bool),
                'scan_time_sec': ParameterValue(LaunchConfiguration('scan_time_sec'), value_type=float),
                'scan_publish_decimation': ParameterValue(LaunchConfiguration('scan_publish_decimation'), value_type=int),
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orbslam_scan_static_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_scan_static_tf')),
            arguments=[
                LaunchConfiguration('scan_x'),
                LaunchConfiguration('scan_y'),
                LaunchConfiguration('scan_z'),
                LaunchConfiguration('scan_yaw'),
                LaunchConfiguration('scan_pitch'),
                LaunchConfiguration('scan_roll'),
                LaunchConfiguration('base_frame'),
                LaunchConfiguration('scan_frame'),
            ],
        ),
    ])
