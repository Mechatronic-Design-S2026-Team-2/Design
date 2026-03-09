from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([
        FindPackageShare('hexapod_mujoco_nav'),
        'config',
        'gait_phase_controller.yaml',
    ])
    return LaunchDescription([
        Node(
            package='hexapod_mujoco_nav',
            executable='gait_phase_controller',
            name='gait_phase_controller',
            output='screen',
            parameters=[params],
        )
    ])
