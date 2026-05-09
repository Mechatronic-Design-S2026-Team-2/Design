from setuptools import find_packages, setup

package_name = 'hexapod_operator_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/operator_interface.launch.py']),
        ('share/' + package_name + '/config', ['config/operator_interface.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexapod',
    maintainer_email='user@example.com',
    description='HTTP operator interface and command arbiter for hexapod Nav2, teleop, estop, map, and battery status.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operator_interface_node = hexapod_operator_interface.operator_interface_node:main',
            'scan_gate_node = hexapod_operator_interface.scan_gate_node:main',
        ],
    },
)
