from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def _default_linkage():
    hardware_prefix = get_package_prefix('hexapod_hardware_cpp')
    hardware_share = get_package_share_directory('hexapod_hardware_cpp')
    ws_root = os.path.dirname(os.path.dirname(hardware_prefix))
    generated = os.path.join(ws_root, 'hexapod_lut_out', 'linkage_precomputed.yaml')
    if os.path.exists(generated):
        return generated
    return os.path.join(hardware_share, 'config', 'linkage_measured.yaml')

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_nav_cpp')
    params = os.path.join(pkg_share, 'config', 'controller_stack.yaml')
    linkage = _default_linkage()

    return LaunchDescription([
        Node(package='hexapod_nav_cpp', executable='klann_mppi_controller_node', name='klann_mppi_controller_node', parameters=[params, {'linkage_yaml': linkage}]),
    ])
