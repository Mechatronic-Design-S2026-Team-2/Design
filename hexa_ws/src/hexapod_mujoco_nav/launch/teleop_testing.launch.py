from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    hold_timeout_sec = LaunchConfiguration('hold_timeout_sec')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.15'),
        DeclareLaunchArgument('turn', default_value='1.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('hold_timeout_sec', default_value='0.20'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),

        Node(
            package='hexapod_mujoco_nav',
            executable='cmd_vel_repeater',
            name='cmd_vel_repeater',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_key',
                'output_topic': '/cmd_vel',
                'publish_rate_hz': publish_rate_hz,
                'hold_timeout_sec': hold_timeout_sec,
                'frame_id': frame_id,
                'force_zero_on_start': True,
            }],
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'stamped': True,
                'frame_id': frame_id,
                'speed': speed,
                'turn': turn,
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel_key'),
            ],
        ),
    ])
