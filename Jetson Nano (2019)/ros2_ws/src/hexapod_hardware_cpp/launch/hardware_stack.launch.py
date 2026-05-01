from launch import LaunchDescription
from launch_ros.actions import Node
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

    return LaunchDescription([
        Node(package='hexapod_hardware_cpp', executable='wt901_imu_node', name='wt901_imu_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='motor_state_aggregator_node', name='motor_state_aggregator_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='phase_command_router_node', name='phase_command_router_node', parameters=[params]),
        Node(package='hexapod_hardware_cpp', executable='body_state_estimator_node', name='body_state_estimator_node', parameters=[params, {'linkage_yaml': linkage}]),
        Node(package='hexapod_hardware_cpp', executable='weighted_odom_fusion_node', name='weighted_odom_fusion_node', parameters=[params]),
    ])
