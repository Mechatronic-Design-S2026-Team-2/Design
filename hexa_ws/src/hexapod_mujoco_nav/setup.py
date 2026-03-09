from setuptools import find_packages, setup

package_name = 'hexapod_mujoco_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [
            'launch/sim_nav.launch.py',
            'launch/gait_phase_controller.launch.py',
            'launch/teleop_testing.launch.py',
        ]),
        (f'share/{package_name}/config', [
            'config/controllers.yaml',
            'config/nav2_params.yaml',
            'config/ekf.yaml',
            'config/gait_phase_controller.yaml',
        ]),
        (f'share/{package_name}/urdf', ['urdf/hexapod_control.urdf.xacro']),
        (f'share/{package_name}/mjcf', ['mjcf/scene.xml']),
        (f'share/{package_name}', ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='MuJoCo + ROS 2 control + Nav2 draft package for the hexapod Klann project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_bridge_node = hexapod_mujoco_nav.mujoco_bridge_node:main',
            'klann_cmdvel_bridge = hexapod_mujoco_nav.klann_cmdvel_bridge:main',
            'gait_phase_controller = hexapod_mujoco_nav.gait_phase_controller:main',
            'cmd_vel_repeater = hexapod_mujoco_nav.cmd_vel_repeater:main',
        ],
    },
)
