# hexapod_control_gui (ROS 2)

A single-process Qt GUI (PyQt5) for a control PC that interfaces with your Jetson hexapod over ROS 2.

Features:
- Holonomic teleop publishing to `/cmd_vel_teleop` (W/S vx, A/D vy, Q/E yaw)
- Mode publish to `/mode/operation` (DIRECT/AUTO_*)
- Rider-present publish to `/mode/rider_present`
- E-stop toggle publish to `/safety/e_stop`
- Gait mode publish to `/gait_mode` (String by default; optional Int32)
- Live status: Battery, Odom pose, IMU yaw, Diagnostics summary
- Live camera views: `/camera/color/image_raw` and `/camera/depth/image_rect_raw`
- Topic presence table for the key topics you listed

## Install (Ubuntu 24.04 / Jazzy control PC)

ROS dependencies (Jazzy):
```bash
sudo apt update
sudo apt install -y   ros-jazzy-rclpy   ros-jazzy-cv-bridge   ros-jazzy-diagnostic-msgs   ros-jazzy-tf2-ros-py
```

GUI deps:
```bash
sudo apt install -y python3-pyqt5 python3-numpy python3-opencv
```

## Build

In your control PC ROS 2 workspace:
```bash
cd ~/ros2_ws/src
# copy hexapod_control_gui/ here
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch hexapod_control_gui hexapod_control_gui.launch.py
```

## Notes

- If `/cmd_vel_teleop` is actually `geometry_msgs/TwistStamped`, the GUI will detect it and publish TwistStamped.
- If the depth image is `16UC1`, it assumes millimeters (RealSense default) and displays a normalized grayscale view, plus center/min distance.
- The GUI is designed to run even if topics are missing (it will show 'no image' / '--').

## Parameters
See `config/gui_defaults.yaml`.
