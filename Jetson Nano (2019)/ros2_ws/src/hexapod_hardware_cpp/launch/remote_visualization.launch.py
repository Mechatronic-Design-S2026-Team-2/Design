import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _default_linkage():
    prefix = get_package_prefix('hexapod_hardware_cpp')
    share = get_package_share_directory('hexapod_hardware_cpp')
    ws_root = os.path.dirname(os.path.dirname(prefix))
    generated = os.path.join(ws_root, 'hexapod_lut_out', 'linkage_precomputed.yaml')
    if os.path.exists(generated):
        return generated
    return os.path.join(share, 'config', 'linkage_measured.yaml')


def _default_rviz_config():
    share = get_package_share_directory('hexapod_hardware_cpp')
    return os.path.join(share, 'config', 'remote_visualization.rviz')


def _launch_visualizer(context, *args, **kwargs):
    raw = LaunchConfiguration('linkage_yaml').perform(context)
    linkage = os.path.abspath(os.path.expandvars(os.path.expanduser(raw if raw else _default_linkage())))
    marker_topic = LaunchConfiguration('marker_topic').perform(context)
    odom_topic = LaunchConfiguration('compact_odometry_topic').perform(context)
    rviz_raw = LaunchConfiguration('rviz_config').perform(context)
    rviz_config = os.path.abspath(os.path.expandvars(os.path.expanduser(rviz_raw if rviz_raw else _default_rviz_config())))

    return [
        Node(
            package='hexapod_hardware_cpp',
            executable='klann_visualizer_node',
            name='klann_visualizer',
            output='screen',
            parameters=[{
                'linkage_yaml': linkage,
                'marker_topic': marker_topic,
                'subscribe_compact_odometry': False,
                'compact_odometry_topic': odom_topic,
                'subscribe_motor_state': True,
                'subscribe_body_state': False,
                'body_state_topic': '/hexapod/body_state',
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'apply_body_pose_transform': False,
                'mirror_model_across_y_axis': True,
                'show_lookup_curves': True,
                'show_bbox': True,
                'animate': False,
                'stamp_markers_zero_time': LaunchConfiguration('stamp_markers_zero_time').perform(context).lower() in ('1', 'true', 'yes'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('open_rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('linkage_yaml', default_value=''),
        DeclareLaunchArgument('marker_topic', default_value='/klann_markers'),
        DeclareLaunchArgument('compact_odometry_topic', default_value='/motor_output_odom'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('stamp_markers_zero_time', default_value='true'),
        OpaqueFunction(function=_launch_visualizer),
    ])
