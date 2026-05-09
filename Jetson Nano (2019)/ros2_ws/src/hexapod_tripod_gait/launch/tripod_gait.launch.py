from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _declare(name, default_value, description):
    return DeclareLaunchArgument(name, default_value=str(default_value), description=description)


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    launch_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hexapod_tripod_gait'),
                'config',
                'tripod_gait.yaml',
            ]),
            description='YAML configuration for the tripod gait node.',
        ),
        _declare('publish_rate_hz', '10.0', 'RPM command publish timer rate; internally clamped to <= 10 Hz.'),
        _declare('linear_rpm_per_mps', '1200.0', 'Open-loop forward RPM scale.'),
        _declare('yaw_rpm_per_radps', '600.0', 'Open-loop yaw RPM scale.'),
        _declare('max_motor_rpm', '1500.0', 'Absolute motor RPM clamp.'),
        _declare('rpm_rate_limit_per_sec', '0.0', 'Optional output slew limit in RPM/s; zero disables.'),
        _declare('phase_lock_enabled', 'true', 'Enable phase odometry tripod synchronization.'),
        _declare('phase_kp_rpm_per_rad', '25.0', 'Tripod phase-lock proportional gain in RPM/rad.'),
        _declare('max_phase_correction_rpm', '100.0', 'Maximum per-leg phase-lock correction in RPM.'),
        _declare('phase_lock_min_linear_rpm', '10.0', 'Minimum linear RPM before phase lock is applied.'),
        _declare('phase_lock_during_pure_turn', 'false', 'Permit phase lock during pure yaw commands.'),
        _declare('imu_feedback_enabled', 'true', 'Master enable for raw sensor_msgs/Imu subscription and raw-IMU feedback.'),
        _declare('euler_feedback_enabled', 'true', 'Master enable for Euler topic subscription.'),
        _declare('yaw_rate_feedback_enabled', 'true', 'Enable yaw-rate feedback when raw IMU is available.'),
        _declare('yaw_rate_feedback_kp', '0.0', 'Yaw-rate feedback gain. Leave zero until yaw sign is verified.'),
        _declare('max_yaw_rate_correction_radps', '0.35', 'Clamp for yaw-rate feedback correction.'),
        _declare('linear_accel_feedback_enabled', 'false', 'Enable forward-velocity estimate from IMU acceleration.'),
        _declare('linear_velocity_feedback_kp', '0.0', 'Gain from commanded-minus-estimated linear velocity to command correction.'),
        _declare('max_linear_velocity_correction_mps', '0.20', 'Clamp for acceleration-derived linear velocity correction.'),
        _declare('linear_accel_axis', '0', 'IMU acceleration axis used as forward acceleration: 0=x, 1=y, 2=z.'),
        _declare('linear_accel_sign', '1.0', 'Sign applied to the selected acceleration axis.'),
        _declare('accel_deadband_mps2', '0.10', 'Acceleration deadband before velocity integration.'),
        _declare('accel_velocity_leak_tau_sec', '1.50', 'Leaky-integrator time constant for acceleration-derived velocity; zero disables leak.'),
        _declare('max_estimated_linear_velocity_mps', '1.00', 'Clamp for acceleration-derived linear velocity estimate.'),
        _declare('tilt_safety_enabled', 'true', 'Enable roll/pitch tilt-based speed scaling.'),
        _declare('use_raw_imu_orientation', 'true', 'Use raw IMU quaternion orientation for roll/pitch if present.'),
        _declare('euler_angles_are_degrees', 'false', 'Set true if Euler feedback topic is in degrees.'),
        _declare('roll_soft_limit_rad', '0.35', 'Roll angle where tilt scaling begins.'),
        _declare('roll_hard_limit_rad', '0.70', 'Roll angle where commanded motion is scaled to zero.'),
        _declare('pitch_soft_limit_rad', '0.35', 'Pitch angle where tilt scaling begins.'),
        _declare('pitch_hard_limit_rad', '0.70', 'Pitch angle where commanded motion is scaled to zero.'),
        _declare('publish_unchanged_commands', 'false', 'Publish only changed RPM vectors when false in direct RPM mode.'),
        _declare('latch_last_teleop_command', 'true', 'Keep the last teleop command active until a new command arrives.'),
        _declare('command_output_mode', 'rpm', "Output mode: 'rpm' publishes MotorRpmArray directly; 'phase' publishes LegPhaseCommand to the hardware router."),
        _declare('use_motor_state_feedback', 'true', 'Use /hexapod/motor_state phase feedback from the hardware aggregator.'),
        _declare('use_compact_odometry_feedback', 'false', 'Use legacy /motor_output_odom phase feedback. Disable for raw-encoder ESP firmware.'),
        _declare('motor_state_topic', '/hexapod/motor_state', 'HexapodMotorState feedback topic in canonical [RF,RM,RB,LF,LM,LB] order.'),
        _declare('motor_state_require_fresh', 'false', 'Require fresh/comm_ok motor_state legs before accepting phase feedback.'),
        _declare('phase_command_topic', 'hexapod/phase_cmd', 'LegPhaseCommand topic consumed by phase_command_router_node.'),
        _declare('gear_ratio', '50.0', 'Motor-to-output gear ratio used for phase velocity commands.'),
        _declare('phase_command_use_phase_velocity', 'true', 'Populate LegPhaseCommand.phase_velocity_rad_s in phase output mode.'),
        _declare('phase_command_use_motor_rpm_ff', 'false', 'Populate LegPhaseCommand.motor_rpm_ff in phase output mode. Do not enable with phase velocity unless intentional.'),
    ]

    launch_overrides = {
        'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
        'linear_rpm_per_mps': ParameterValue(LaunchConfiguration('linear_rpm_per_mps'), value_type=float),
        'yaw_rpm_per_radps': ParameterValue(LaunchConfiguration('yaw_rpm_per_radps'), value_type=float),
        'max_motor_rpm': ParameterValue(LaunchConfiguration('max_motor_rpm'), value_type=float),
        'rpm_rate_limit_per_sec': ParameterValue(LaunchConfiguration('rpm_rate_limit_per_sec'), value_type=float),
        'phase_lock_enabled': ParameterValue(LaunchConfiguration('phase_lock_enabled'), value_type=bool),
        'phase_kp_rpm_per_rad': ParameterValue(LaunchConfiguration('phase_kp_rpm_per_rad'), value_type=float),
        'max_phase_correction_rpm': ParameterValue(LaunchConfiguration('max_phase_correction_rpm'), value_type=float),
        'phase_lock_min_linear_rpm': ParameterValue(LaunchConfiguration('phase_lock_min_linear_rpm'), value_type=float),
        'phase_lock_during_pure_turn': ParameterValue(LaunchConfiguration('phase_lock_during_pure_turn'), value_type=bool),
        'imu_feedback_enabled': ParameterValue(LaunchConfiguration('imu_feedback_enabled'), value_type=bool),
        'euler_feedback_enabled': ParameterValue(LaunchConfiguration('euler_feedback_enabled'), value_type=bool),
        'yaw_rate_feedback_enabled': ParameterValue(LaunchConfiguration('yaw_rate_feedback_enabled'), value_type=bool),
        'yaw_rate_feedback_kp': ParameterValue(LaunchConfiguration('yaw_rate_feedback_kp'), value_type=float),
        'max_yaw_rate_correction_radps': ParameterValue(LaunchConfiguration('max_yaw_rate_correction_radps'), value_type=float),
        'linear_accel_feedback_enabled': ParameterValue(LaunchConfiguration('linear_accel_feedback_enabled'), value_type=bool),
        'linear_velocity_feedback_kp': ParameterValue(LaunchConfiguration('linear_velocity_feedback_kp'), value_type=float),
        'max_linear_velocity_correction_mps': ParameterValue(LaunchConfiguration('max_linear_velocity_correction_mps'), value_type=float),
        'linear_accel_axis': ParameterValue(LaunchConfiguration('linear_accel_axis'), value_type=int),
        'linear_accel_sign': ParameterValue(LaunchConfiguration('linear_accel_sign'), value_type=float),
        'accel_deadband_mps2': ParameterValue(LaunchConfiguration('accel_deadband_mps2'), value_type=float),
        'accel_velocity_leak_tau_sec': ParameterValue(LaunchConfiguration('accel_velocity_leak_tau_sec'), value_type=float),
        'max_estimated_linear_velocity_mps': ParameterValue(LaunchConfiguration('max_estimated_linear_velocity_mps'), value_type=float),
        'tilt_safety_enabled': ParameterValue(LaunchConfiguration('tilt_safety_enabled'), value_type=bool),
        'use_raw_imu_orientation': ParameterValue(LaunchConfiguration('use_raw_imu_orientation'), value_type=bool),
        'euler_angles_are_degrees': ParameterValue(LaunchConfiguration('euler_angles_are_degrees'), value_type=bool),
        'roll_soft_limit_rad': ParameterValue(LaunchConfiguration('roll_soft_limit_rad'), value_type=float),
        'roll_hard_limit_rad': ParameterValue(LaunchConfiguration('roll_hard_limit_rad'), value_type=float),
        'pitch_soft_limit_rad': ParameterValue(LaunchConfiguration('pitch_soft_limit_rad'), value_type=float),
        'pitch_hard_limit_rad': ParameterValue(LaunchConfiguration('pitch_hard_limit_rad'), value_type=float),
        'publish_unchanged_commands': ParameterValue(LaunchConfiguration('publish_unchanged_commands'), value_type=bool),
        'latch_last_teleop_command': ParameterValue(LaunchConfiguration('latch_last_teleop_command'), value_type=bool),
        'command_output_mode': ParameterValue(LaunchConfiguration('command_output_mode'), value_type=str),
        'use_motor_state_feedback': ParameterValue(LaunchConfiguration('use_motor_state_feedback'), value_type=bool),
        'use_compact_odometry_feedback': ParameterValue(LaunchConfiguration('use_compact_odometry_feedback'), value_type=bool),
        'motor_state_topic': ParameterValue(LaunchConfiguration('motor_state_topic'), value_type=str),
        'motor_state_require_fresh': ParameterValue(LaunchConfiguration('motor_state_require_fresh'), value_type=bool),
        'phase_command_topic': ParameterValue(LaunchConfiguration('phase_command_topic'), value_type=str),
        'gear_ratio': ParameterValue(LaunchConfiguration('gear_ratio'), value_type=float),
        'phase_command_use_phase_velocity': ParameterValue(LaunchConfiguration('phase_command_use_phase_velocity'), value_type=bool),
        'phase_command_use_motor_rpm_ff': ParameterValue(LaunchConfiguration('phase_command_use_motor_rpm_ff'), value_type=bool),
    }

    return LaunchDescription(launch_arguments + [
        Node(
            package='hexapod_tripod_gait',
            executable='tripod_gait_node',
            name='hexapod_tripod_gait',
            output='screen',
            parameters=[config_file, launch_overrides],
        ),
    ])
