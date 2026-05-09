# hexapod_nav2_bringup

Nav2 bringup for the integrated hexapod ROS 2 stack.

This package expects the uploaded host-side packages to be present:

- `dsy_motor_msgs`
- `hexapod_control_interfaces`
- `hexapod_hardware_cpp`
- `hexapod_orbslam_udp_bridge`
- `hexapod_tripod_gait`

Default launch chain:

```text
Nav2 controller_server -> velocity_smoother -> /cmd_vel
hexapod_tripod_gait -> hexapod/phase_cmd
hexapod_hardware_cpp/phase_command_router_node -> /motor_rpm_cmd
ESP32 micro-ROS bridge -> /motor_output_odom
hexapod_hardware_cpp/motor_state_aggregator_node -> /hexapod/motor_state
hexapod_hardware_cpp/body_state_estimator_node -> /hexapod/kinematic_odom
hexapod_hardware_cpp/weighted_odom_fusion_node -> /odom + odom->base_link
slam_toolbox -> map->odom + /map
```

The tripod controller is launched with `command_output_mode:=phase`, so it interfaces through `hexapod_control_interfaces/msg/LegPhaseCommand` instead of bypassing the host hardware stack.

Launch with ORB-SLAM bridge and slam_toolbox already running:

```bash
ros2 launch hexapod_nav2_bringup hexapod_nav2.launch.py \
  start_orbslam_slamtoolbox:=false \
  start_hardware_stack:=true \
  start_tripod_controller:=true
```

Launch all container-side pieces:

```bash
ros2 launch hexapod_nav2_bringup hexapod_nav2.launch.py \
  start_orbslam_slamtoolbox:=true \
  start_hardware_stack:=true \
  start_tripod_controller:=true
```

Sanity checks:

```bash
ros2 topic hz /scan
ros2 topic hz /hexapod/orbslam_odom
ros2 topic hz /hexapod/motor_state
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /motor_rpm_cmd
```
