# hexapod_mujoco_nav

## Purpose

MuJoCo + ROS 2 Jazzy scaffold for:

- hexapod / Klann simulation
- RGB-D + IMU simulation
- odom / TF publishing
- `/cmd_vel` teleop / Nav2 interface

## Assumptions

- ROS 2 Jazzy already installed and sourceable

## Expected MJCF Names

Bridge expects these names in MJCF:

- body: `base_link`
- camera: `rgbd_cam`
- sensors: `imu_accel`, `imu_gyro`, `imu_quat`

## First-Time Dependency Install

```bash
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard

python3 -m pip install --break-system-packages -U mujoco

```

Optional:

```bash
sudo apt install -y ros-jazzy-robot-localization
```

## ros2_control Setup

Recommended if using controller-manager path instead of direct MuJoCo joint-command path.

Install topic-based hardware interfaces:

```bash
cd ~/hexa_ws/src
git clone https://github.com/ros-controls/topic_based_hardware_interfaces.git

cd ~/hexa_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select joint_state_topic_hardware_interface cm_topic_hardware_component --symlink-install
source install/setup.bash
```

## Workspace Build

```bash
cd ~/hexa_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hexapod_mujoco_nav
source install/setup.bash
```

## Launch MuJoCo + ROS Bridge

Viewer on:

```bash
cd ~/hexa_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch hexapod_mujoco_nav sim_nav.launch.py \
  model_path:=/home/user/hexa_ws/src/hexapod_mujoco_nav/mjcf/scene.xml \
  viewer:=true \
  use_ros2_control:=false \
  use_ekf:=false
```

## Launch Keyboard Teleop

New terminal:

```bash
cd ~/hexa_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  --remap cmd_vel:=/cmd_vel
```

## Teleop Keys

- `i` forward
- `,` backward
- `j` left yaw
- `l` right yaw
- `k` stop
- `q/z`, `w/x`, `e/c` speed scale

## Control Path

```text
/cmd_vel -> klann_cmdvel_bridge -> MuJoCo actuator commands
```
Use teleop or another ROS publisher. GUI sliders will not affect robot.

## Optional EKF

If `robot_localization` installed:

```bash
ros2 launch hexapod_mujoco_nav sim_nav.launch.py \
  model_path:=/home/user/hexa_ws/src/hexapod_mujoco_nav/mjcf/scene.xml \
  viewer:=true \
  use_ros2_control:=false \
  use_ekf:=true
```

## Nav2 Boundary

This package stops at the simulation / interface boundary.

Provided / expected interfaces:

- input: `/cmd_vel`
- output: `/odom`
- output: `/tf`
- output: `/imu/data`
- output: `/camera/depth/points`
