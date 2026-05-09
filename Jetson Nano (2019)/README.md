# Hexapod ROS 2 Workspace

This workspace contains the Jetson-side ROS 2 Jazzy stack for the Klann-linkage hexapod. It is intended to live at `/ros2_ws` inside the existing ROS 2 container. The archive intentionally excludes `build/`, `install/`, and `log/` so it can be copied onto the Jetson and rebuilt in place.

The current stack assumes the RealSense D415 and ORB-SLAM2 process run on the bare-metal Jetson, while ROS 2, Nav2, SLAM Toolbox, the browser operator interface, hardware aggregation, and tripod gait controller run inside the `jazzy` container. The ESP32/micro-ROS bridge is expected to be provided separately by the `microros_agent` container.

## Current system architecture

### Bare-metal Jetson

`orbslam2-realsense.service` owns the RealSense D415 and runs the CUDA ORB-SLAM2 RGB-D frontend. It exports pose, map points, and a virtual planar scan to UDP port `5005`.

The Jetson also runs host-side systemd helpers:

- `hexapod-ros2-full-launch.service`: executes the full ROS 2 launch inside the already-running `jazzy` container.
- `hexapod-host-control.service`: exposes limited localhost-only HTTP actions for the browser GUI, such as restarting ORB-SLAM2 or the `microros_agent` container.

### ROS 2 container

The main runtime chain is:

```text
ORB-SLAM2 bare-metal UDP packets
  -> hexapod_orbslam_udp_bridge
  -> /hexapod/orbslam_odom
  -> /orbslam/map_points
  -> /scan_raw

/scan_raw
  -> scan_gate_node
  -> /scan      for SLAM Toolbox mapping
  -> /scan_nav  for Nav2 local obstacle checking

ESP32/micro-ROS motor odometry
  -> /motor_output_odom
  -> motor_state_aggregator_node
  -> /hexapod/motor_state

/hexapod/motor_state + linkage LUT
  -> body_state_estimator_node
  -> /hexapod/kinematic_odom

/hexapod/orbslam_odom + /hexapod/kinematic_odom + IMU
  -> weighted_odom_fusion_node
  -> /odom and odom -> base_link

/scan + odom -> base_link
  -> slam_toolbox
  -> /map and map -> odom

Browser teleop or Nav2 /cmd_vel_nav
  -> operator_interface_node
  -> /hexapod/cmd_vel_selected
  -> hexapod_tripod_gait
  -> /hexapod/phase_cmd
  -> phase_command_router_node
  -> /motor_rpm_cmd
  -> ESP32 / RS485 drives
```

The TF ownership is:

```text
slam_toolbox:                 map -> odom
weighted_odom_fusion_node:    odom -> base_link
static_transform_publisher:   base_link -> orbslam_scan_frame
```

Do not let ORB-SLAM and SLAM Toolbox both publish `map -> odom`. The current stack uses ORB-SLAM for visual odometry and virtual scan generation, then uses SLAM Toolbox to own the 2D map correction transform.

## Operator interface

The browser GUI is served by `hexapod_operator_interface`.

Default URL:

```text
http://192.168.50.1:8080/
```

Main functions:

- Estop, teleop, and waypoint mode selection.
- Browser teleop with `q/w/e/a/s/d/z/x/c`-style directional commands.
- Latched teleop command behavior, with stop via `s`, space, or the stop/estop control.
- Click-to-waypoint map interaction through Nav2.
- Occupancy grid display with pan and wheel zoom.
- To-scale birdseye hexapod skeleton overlay.
- Battery-voltage display from motor-state / motor-odom topics.
- Map save/load under `/ros2_ws/maps`.
- Mapping-enable toggle through `/hexapod/operator/map_updates_enabled`.
- Host-control buttons for limited restart actions.

Default GUI speeds:

```text
teleop linear speed:       0.50 m/s
teleop angular speed:      0.50 rad/s
teleop max linear speed:   3.00 m/s
teleop max angular speed:  3.00 rad/s
waypoint max linear speed: 0.50 m/s
waypoint max angular speed:0.50 rad/s
```

The operator arbiter publishes only one selected velocity stream to `/hexapod/cmd_vel_selected`. Nav2, teleop, and estop should not publish directly to the gait controller.

## Mapping and Nav2 behavior

The ORB-SLAM UDP bridge publishes virtual scan data on `/scan_raw`. The scan gate splits that into two streams:

```text
/scan_raw -> /scan      gated, restamped, used by SLAM Toolbox
/scan_raw -> /scan_nav  always live, restamped, used by Nav2 local costmap
```

Current mapping defaults:

```text
ORB-SLAM UDP port:                 5005
ORB-SLAM position scale:           1.214
ORB-SLAM scan range scale:         1.214
ORB-SLAM scan publish decimation:  3
scan gate mapping output rate:     about 3 Hz
scan gate nav output rate:         about 10 Hz
scan gate restamp outgoing scans:  true
SLAM Toolbox scan topic:           /scan
Nav2 local obstacle scan topic:    /scan_nav
```

The scan restamp behavior is intentional. The virtual scan is produced outside the container by ORB-SLAM and may carry timestamps that do not line up cleanly with the current TF cache. Restamping at the scan gate has been the more reliable path for SLAM Toolbox occupancy-grid generation on this Jetson setup.

Nav2 is configured for conservative but usable waypoint motion:

```text
Regulated Pure Pursuit desired linear velocity: 0.50 m/s
Regulated Pure Pursuit max angular velocity:   0.50 rad/s
velocity smoother max velocity:                [0.50, 0.0, 0.50]
velocity smoother max acceleration:            [1.00, 0.0, 1.20]
progress checker movement radius:              0.01 m
progress checker movement allowance:           60 s
```

The local costmap uses live `/scan_nav`; the global costmap primarily uses the static SLAM map plus inflation to reduce scan raytracing load.

## LCD1602 status display

The LCD node is in `hexapod_hardware_cpp` and is launched by default from the hardware stack. The current default is intentionally static URL-only output to avoid later character corruption from repeated LCD page updates.

Defaults:

```text
I2C device:     /dev/i2c-1
I2C address:    39 decimal / 0x27 hex
Displayed text: http://192.168.50.1:8080/
```

The node is non-fatal. If the I2C device or LCD backpack is unavailable, the rest of the launch should continue.

## Package overview

### `hexapod_hardware_cpp`

Hardware aggregation and state-estimation package.

Important nodes:

- `wt901_imu_node`: reads the WT901 IMU on `/dev/ttyTHS1` at 9600 baud and publishes raw IMU/euler topics.
- `motor_state_aggregator_node`: consumes compact ESP/micro-ROS motor odometry and publishes canonical `/hexapod/motor_state`.
- `body_state_estimator_node`: estimates body motion from motor phases and precomputed Klann linkage geometry.
- `weighted_odom_fusion_node`: publishes `/odom` and `odom -> base_link` using fused ORB-SLAM / kinematic / IMU information.
- `phase_command_router_node`: converts model-space `/hexapod/phase_cmd` into physical `/motor_rpm_cmd` messages for the ESP32 bridge.
- `lcd1602_status_node`: displays the operator URL on the I2C LCD1602 module.
- `klann_visualizer_node`: publishes marker geometry for the hexapod skeleton visualization.

Key topics:

```text
/motor_output_odom       input from ESP32/micro-ROS
/hexapod/motor_state     canonical motor state
/hexapod/kinematic_odom  kinematic odometry estimate
/hexapod/fused_odom      fused odometry
/odom                    Nav2/SLAM odometry topic
/hexapod/phase_cmd       model-space gait command
/motor_rpm_cmd           physical motor RPM command
/imu/data_raw
/imu/euler
```

### `hexapod_orbslam_udp_bridge`

ROS 2 bridge for the bare-metal ORB-SLAM2 frontend.

Functions:

- Listens on UDP port `5005`.
- Publishes `/hexapod/orbslam_odom`.
- Publishes sparse map points on `/orbslam/map_points`.
- Publishes virtual scan data on `/scan_raw`.
- Provides `orbslam_slam_toolbox.launch.py` for bridge + static scan TF + SLAM Toolbox.

Default frame and scale values preserve the known-good setup:

```text
map frame:             map
scan frame:            orbslam_scan_frame
scan x offset:         0.2794 m
position scale:        1.214
scan range scale:      1.214
axis conversion:       false by default for current ORB-SLAM output
packet stamp usage:    false by default
```

### `hexapod_operator_interface`

Browser operator UI, command arbiter, map utilities, and scan gate.

Important executables:

- `operator_interface_node`: HTTP GUI, operator mode state machine, teleop/waypoint command selection, battery display, map save/load endpoints.
- `scan_gate_node`: splits `/scan_raw` into `/scan` for mapping and `/scan_nav` for Nav2.

Key topics:

```text
/hexapod/cmd_vel_selected
/hexapod/operator/battery_voltage
/hexapod/operator/status_json
/hexapod/operator/map_updates_enabled
/hexapod/operator/scan_gate_status
/hexapod/operator/loaded_map
/scan_raw
/scan
/scan_nav
```

### `hexapod_tripod_gait`

C++ tripod gait command generator.

Current integrated-stack behavior:

- Subscribes to `/hexapod/cmd_vel_selected`.
- Uses `/hexapod/motor_state` for phase feedback.
- Publishes `/hexapod/phase_cmd`.
- Can latch the last command when configured.
- Outputs model-space phase commands rather than directly publishing RPM in the integrated hardware stack.

Leg order used internally:

```text
[RB, RM, RF, LF, LM, LB]
```

The hardware motor-state order is canonicalized by `hexapod_hardware_cpp`, with mappings preserved in the launch/config files.

### `hexapod_nav2_bringup`

Integrated bringup package for the full container-side stack.

Functions:

- Starts ORB-SLAM UDP bridge and SLAM Toolbox.
- Starts the hardware stack.
- Starts the scan gate.
- Starts the browser operator interface.
- Starts the tripod gait controller in integrated phase-command mode.
- Starts Nav2 controller/planner/behavior/BT/smoother/waypoint lifecycle nodes.
- Provides tuned Nav2 parameters for the hexapod path.

The full-stack launch is:

```bash
ros2 launch hexapod_nav2_bringup hexapod_nav2.launch.py
```

### `hexapod_control_interfaces`

Custom message definitions used between hardware, gait, state-estimation, and visualization nodes.

Primary messages:

```text
HexapodMotorState.msg
LegPhaseCommand.msg
KlannBodyState.msg
LegActuationState.msg
```

### `dsy_motor_msgs`

Motor command and odometry messages used by the ESP32/micro-ROS side.

This package contains the compact motor odometry and RPM command interfaces used by the ROS-side hardware aggregation package.

### `hexapod_nav_cpp`

Earlier C++ Nav2/controller-plugin work. It remains in the workspace for reference and future controller-plugin development, but the current operator/hardware path primarily uses `hexapod_tripod_gait` plus `hexapod_nav2_bringup`.

## Build inside the existing ROS 2 container

Full workspace build:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Common incremental build after edits:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  hexapod_hardware_cpp \
  hexapod_orbslam_udp_bridge \
  hexapod_operator_interface \
  hexapod_tripod_gait \
  hexapod_nav2_bringup
source install/setup.bash
```

If custom message definitions changed, build through dependent packages instead of rebuilding only one downstream package.

## Manual full launch

```bash
ros2 launch hexapod_nav2_bringup hexapod_nav2.launch.py \
  start_orbslam_slamtoolbox:=true \
  start_hardware_stack:=true \
  start_tripod_controller:=true \
  start_operator_interface:=true \
  operator_http_port:=8080 \
  operator_default_mode:=estop \
  operator_latch_teleop_commands:=true \
  operator_keyboard_linear_mps:=0.50 \
  operator_keyboard_angular_radps:=0.50 \
  operator_max_linear_mps:=3.00 \
  operator_max_angular_radps:=3.00 \
  operator_waypoint_max_linear_mps:=0.50 \
  operator_waypoint_max_angular_radps:=0.50 \
  orbslam_scan_publish_decimation:=3 \
  scan_gate_restamp_scans:=true \
  orbslam_stamp_scan_with_latest_pose:=false
```

The browser UI should come up in estop mode. Movement should require an operator mode change from the GUI.

## Host autostart

The ROS 2 container is assumed to already start automatically. The provided host-side service only runs the full ROS launch inside that existing container.

Install or update autostart on the Jetson host:

```bash
cd /ros2_ws
sudo ./scripts/install_hexapod_autostart.sh
sudo systemctl daemon-reload
sudo systemctl restart hexapod-ros2-full-launch.service
sudo journalctl -u hexapod-ros2-full-launch.service -f
```

The service uses `docker exec` against the configured container name, normally `jazzy`.

Configuration defaults are installed under:

```text
/etc/default/hexapod-ros2-full-launch
```

## Host-control helper

The host-control helper is optional but needed for GUI restart buttons.

Install it on the Jetson host:

```bash
cd /ros2_ws
sudo ./scripts/install_hexapod_host_control.sh
curl http://127.0.0.1:18080/status
```

The helper is intentionally narrow. It is intended to restart only known host services/containers such as:

```text
orbslam2-realsense.service
microros_agent Docker container
```

## Common verification commands

Topic rates:

```bash
ros2 topic hz /scan_raw
ros2 topic hz /scan
ros2 topic hz /scan_nav
ros2 topic hz /map
ros2 topic hz /odom
ros2 topic hz /hexapod/motor_state
```

Command path:

```bash
ros2 topic echo /hexapod/cmd_vel_selected
ros2 topic echo /hexapod/phase_cmd
ros2 topic echo /motor_rpm_cmd
```

Mapping and scan gate:

```bash
ros2 topic echo /hexapod/operator/scan_gate_status
ros2 topic echo /hexapod/operator/map_updates_enabled
ros2 run tf2_ros tf2_echo map base_link
```

Operator GUI:

```bash
curl http://192.168.50.1:8080/api/state
```

LCD:

```bash
ls /dev/i2c-*
i2cdetect -y -r 1
```

## Expected key topics

```text
/map
/map_updates
/scan_raw
/scan
/scan_nav
/odom
/tf
/tf_static
/hexapod/orbslam_odom
/hexapod/motor_state
/hexapod/kinematic_odom
/hexapod/fused_odom
/hexapod/cmd_vel_selected
/hexapod/phase_cmd
/motor_rpm_cmd
/motor_output_odom
/imu/data_raw
/imu/euler
/hexapod/operator/status_json
/hexapod/operator/battery_voltage
/hexapod/operator/scan_gate_status
```

## Runtime troubleshooting

If `/scan_raw`, `/scan`, and `/scan_nav` publish but `/map` does not publish, check SLAM Toolbox scan acceptance and TF:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /hexapod/operator/scan_gate_status
```

If GUI teleop changes mode but no motor command appears, check the command chain:

```bash
ros2 param get /hexapod_tripod_gait cmd_vel_topic
ros2 topic echo /hexapod/cmd_vel_selected
ros2 topic echo /hexapod/phase_cmd
ros2 topic echo /motor_rpm_cmd
```

Expected gait parameter:

```text
/hexapod/cmd_vel_selected
```

If `/motor_output_odom` is live but `/hexapod/motor_state` is not, check the hardware aggregator and message compatibility. If `/hexapod/motor_state` is live but phase feedback is rejected, verify the gait node's motor-state strictness parameters and array completeness.

If Nav2 waypoint commands are too slow, compare the stages:

```bash
ros2 topic echo /cmd_vel_nav_raw
ros2 topic echo /cmd_vel_nav
ros2 topic echo /hexapod/cmd_vel_selected
ros2 topic echo /hexapod/phase_cmd
```

This identifies whether the limit is coming from Nav2, the velocity smoother, the operator arbiter, or the tripod gait conversion.

If Fast DDS shared-memory lock errors appear in Docker, the autostart script sets:

```text
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

This avoids the repeated `fastrtps_port... open_and_lock_file failed` errors in the host-networked container setup.

## Offline Jetson note

Do not rebuild the Docker image on the offline Jetson unless the required apt packages are already cached or the image was built on another machine and transferred over. This workspace contains source and host deployment scripts only. It does not contain compiled ROS build products.
