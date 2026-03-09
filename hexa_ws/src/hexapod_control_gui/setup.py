from setuptools import setup

package_name = 'hexapod_control_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hexapod_control_gui.launch.py']),
        ('share/' + package_name + '/config', ['config/gui_defaults.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 control PC GUI for hexapod.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hexapod_control_gui = hexapod_control_gui.main:main',
        ],
    },
)
