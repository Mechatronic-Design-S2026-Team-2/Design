from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('hexapod_mujoco_nav')

    model_path = LaunchConfiguration('model_path')
    viewer = LaunchConfiguration('viewer')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ekf = LaunchConfiguration('use_ekf')

    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'hexapod_control.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])
    ekf_params = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([pkg, 'mjcf', 'scene.xml']),
            description='Path to MuJoCo scene.xml',
        ),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            condition=IfCondition(use_ros2_control),
            parameters=[
                {'robot_description': robot_description, 'use_sim_time': True},
                controllers_yaml,
            ],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_ros2_control),
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['leg_velocity_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_ros2_control),
        ),

        Node(
            package='hexapod_mujoco_nav',
            executable='mujoco_bridge_node',
            name='mujoco_bridge_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'use_sim_time': False,
                'viewer': viewer,
                'sim_hz': 400.0,
                'render_hz': 20.0,
                'point_stride': 4,
                'camera_name': 'rgbd_cam',
                'base_body': 'base_link',
                'frame_id': 'base_link',
                'odom_frame': 'odom',
                'camera_frame': 'camera_link',
                'camera_optical_frame': 'camera_optical_frame',
                'imu_frame': 'imu_link',
                'camera_width': 640,
                'camera_height': 480,
                'camera_fovy_deg': 69.0,
            }],
        ),

        Node(
            package='hexapod_mujoco_nav',
            executable='klann_cmdvel_bridge',
            name='klann_cmdvel_bridge',
            output='screen',
            parameters=[{
                'cmd_topic': '/cmd_vel',
                # This must match the direct JointState command topic
                # that mujoco_bridge_node subscribes to.
                'joint_command_topic': '/topic_based_joint_commands',
                'forward_gain': 4.0,
                'yaw_gain': 2.0,
                'max_leg_speed': 8.0,
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_ekf),
            parameters=[ekf_params, {'use_sim_time': True}],
        ),
    ])
