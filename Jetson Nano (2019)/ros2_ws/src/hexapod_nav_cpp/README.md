# hexapod_nav_cpp — hardware-safe MPPI phase controller

This package is a drop-in controller folder for the current hexapod ROS 2 workspace.  It publishes `hexapod_control_interfaces/msg/LegPhaseCommand` on `/hexapod/phase_cmd`; the existing `phase_command_router_node` remains responsible for converting model-frame leg phase commands to physical `/motor_rpm_cmd` in ESP wire order.

## Current control path

```text
/plan
  -> klann_mppi_controller_node
  -> /hexapod/phase_cmd
  -> phase_command_router_node
  -> /motor_rpm_cmd
  -> ESP32 sweep bridge
```

Feedback path:

```text
/motor_output_odom
  -> motor_state_aggregator_node
  -> /hexapod/motor_state
  -> body_state_estimator_node
  -> /hexapod/body_state
  -> klann_mppi_controller_node
```

The standalone MPPI node does not require ORB-SLAM, `/odom`, `/map`, or `slam_toolbox`.  It consumes `/hexapod/body_state` and `/plan`.

## Hardware-safe changes

Compared with the earlier MPPI package, this version uses conservative defaults that matched the stable bringup behavior:

- `nominal_phase_rate_rad_s = 0.25`
- `phase_rate_limit_rad_s = 0.45`
- `linear_speed_limit_mps = 0.03`
- `angular_speed_limit_rps = 0.08`
- low MPPI perturbation noise
- high phase-sync and control-smoothness penalties
- `forward_phase_rate_sign = -1.0`
- `mirror_kinematics_across_y_axis = true`

It also adds `lock_forward_phase_rate_sign`.  During near-straight forward walking, MPPI samples are not allowed to reverse individual leg phase-rate signs.  This prevents the observed leg direction thrashing and reduces risk of single-turn encoder unwrap errors.

## Build

Only this package needs rebuilding when replacing this folder:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hexapod_nav_cpp
source install/setup.bash
```

No rebuild is required for `dsy_motor_msgs`, `hexapod_hardware_cpp`, `hexapod_orbslam_udp_bridge`, or the ESP firmware unless those packages were changed separately.

## Launch

Start the hardware stack first and wait for the startup pose to complete:

```bash
ros2 launch hexapod_hardware_cpp hardware_stack.launch.py
```

Then run the controller:

```bash
ros2 launch hexapod_nav_cpp controller_stack.launch.py
```

Publish a short local test path in `hexapod_kinematic_odom` coordinates:

```bash
ros2 topic pub --once /plan nav_msgs/msg/Path "{
  header: {frame_id: 'hexapod_kinematic_odom'},
  poses: [
    {header: {frame_id: 'hexapod_kinematic_odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'hexapod_kinematic_odom'}, pose: {position: {x: 0.45, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]
}"
```

Stop by publishing an empty plan:

```bash
ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: 'hexapod_kinematic_odom'}, poses: []}"
```

## Main tuning knobs

`lock_forward_phase_rate_sign: true` should remain enabled for real hardware until the encoder/RS485 path is proven robust under higher rates.

Increase speed in this order:

1. `nominal_phase_rate_rad_s`
2. `phase_rate_limit_rad_s`
3. `linear_speed_limit_mps`

Do not increase MPPI noise first.  Increasing `phase_rate_noise_std` before the gait is stable reintroduces leg reversals.
