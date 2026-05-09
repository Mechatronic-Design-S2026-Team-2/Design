# hexapod_tripod_gait

Simple ROS 2 C++ bridge from standard teleop velocity topics to the six-motor DSY micro-ROS RPM interface.

Leg order is fixed to:

```text
[0] right_back
[1] right_middle
[2] right_front
[3] left_front
[4] left_middle
[5] left_back
```

Forward RPM convention:

```text
right legs: positive RPM for forward
left legs:  negative RPM for forward
forward_motor_signs = [+, +, +, -, -, -]
```

The node publishes `dsy_motor_msgs/msg/MotorRpmArray` on `motor_rpm_cmd` and subscribes to `dsy_motor_msgs/msg/MotorOutputOdometryArray` on `motor_output_odom` using best-effort QoS. It accepts both `geometry_msgs/msg/Twist` on `cmd_vel` and `geometry_msgs/msg/TwistStamped` on `cmd_vel_stamped`; set either topic parameter to an empty string to disable it.

By default, `latch_last_teleop_command: true` keeps the last received teleop command active indefinitely. The teleop app does not need to publish a continuous stream. Send an explicit zero `Twist` to stop the robot. By default, `publish_unchanged_commands: false` suppresses repeated publication of the same quantized six-motor RPM vector.

Tripod grouping:

```text
tripod A = [right_middle, left_front, left_back] = indices [1, 3, 5]
tripod B = [right_back, right_front, left_middle] = indices [0, 2, 4]
```

The phase-lock term uses `output_phase_rad` to keep tripod B approximately pi radians offset from tripod A while translating. Pure yaw defaults to no phase-lock correction because opposite-side turning can make a single common phase reference fight the turn command.

## Feedback inputs

The node can subscribe to:

```text
imu_topic          sensor_msgs/msg/Imu              default: imu/data_raw
euler_topic        geometry_msgs/msg/Vector3Stamped default: imu/euler
euler_vector_topic geometry_msgs/msg/Vector3        default: disabled
```

`imu_feedback_enabled` is the master switch for the raw IMU subscription. If it is false, yaw-rate feedback, acceleration-derived velocity feedback, and raw-IMU quaternion tilt feedback are unavailable.

`euler_feedback_enabled` is the switch for the explicit Euler topic subscription. Euler feedback uses `x=roll`, `y=pitch`, `z=yaw`; set `euler_angles_are_degrees: true` if the topic is in degrees.

Raw IMU yaw feedback uses `angular_velocity.z`. Keep `yaw_rate_feedback_kp` at zero until the yaw-rate sign is verified.

Raw IMU acceleration feedback uses one selected linear acceleration axis, integrates it into a leaky forward-velocity estimate, then compares that estimate against `cmd_vel.linear.x`:

```text
velocity_error = commanded_linear_x - estimated_linear_velocity
linear_correction = linear_velocity_feedback_kp * velocity_error
adjusted_linear_x = commanded_linear_x + clamp(linear_correction)
```

It is disabled by default because accelerometer-only velocity estimates drift. Before enabling it, verify `linear_accel_axis`, `linear_accel_sign`, and the IMU mounting. Start with a low gain.

## Raw-encoder ESP firmware / hardware-stack feedback

For the current ESP firmware, `motor_output_odom.output_phase_rad[]` is no longer a usable
output-shaft phase signal; it carries raw single-turn encoder counts. Run the
hardware stack's `motor_state_aggregator_node` and feed this controller from:

```text
/hexapod/motor_state
```

That topic is `hexapod_control_interfaces/msg/HexapodMotorState` in canonical order:

```text
[RF, RM, RB, LF, LM, LB]
```

This controller still keeps its internal command order:

```text
[RB, RM, RF, LF, LM, LB]
```

so it uses the default mapping:

```yaml
internal_to_motor_state_index: [2, 1, 0, 3, 4, 5]
```

With motor-state feedback, keep:

```yaml
use_motor_state_feedback: true
use_compact_odometry_feedback: false
phase_direction_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
phase_correction_motor_signs: [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]
```

The all-positive `phase_direction_signs` are intentional: the hardware aggregator already
applies the raw-count anchoring, side signs, and canonical leg ordering. The separate
`phase_correction_motor_signs` convert model phase correction back to physical motor RPM.

When `phase_command_router_node` is running, use phase-command output:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py \
  command_output_mode:=phase \
  use_motor_state_feedback:=true \
  use_compact_odometry_feedback:=false
```

In this mode the teleop node publishes ROS-level `hexapod/phase_cmd` keepalives at up to
10 Hz so the router does not time out; the router still suppresses unchanged
`/motor_rpm_cmd` writes to avoid wasting ESP/RS485 bus bandwidth.

## Build

From a ROS 2 workspace:

```bash
mkdir -p ~/hexapod_tripod_ws/src
cp -r dsy_motor_msgs hexapod_tripod_gait ~/hexapod_tripod_ws/src/
cd ~/hexapod_tripod_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dsy_motor_msgs hexapod_tripod_gait
source install/setup.bash
```

If your workspace already has `dsy_motor_msgs` with the same message definitions, copy only `hexapod_tripod_gait` into `src` and build that package against your existing message package.

## Launch

Default launch:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py
```

Turn off all raw IMU feedback and the explicit Euler topic feedback from launch:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py \
  imu_feedback_enabled:=false \
  euler_feedback_enabled:=false
```

Tune phase-lock gain from launch:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py \
  phase_kp_rpm_per_rad:=15.0 \
  max_phase_correction_rpm:=60.0
```

Enable yaw-rate feedback after confirming IMU yaw-rate sign:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py \
  imu_feedback_enabled:=true \
  yaw_rate_feedback_enabled:=true \
  yaw_rate_feedback_kp:=0.10 \
  max_yaw_rate_correction_radps:=0.25
```

Enable acceleration-derived linear velocity feedback cautiously:

```bash
ros2 launch hexapod_tripod_gait tripod_gait.launch.py \
  imu_feedback_enabled:=true \
  linear_accel_feedback_enabled:=true \
  linear_velocity_feedback_kp:=0.10 \
  max_linear_velocity_correction_mps:=0.05 \
  linear_accel_axis:=0 \
  linear_accel_sign:=1.0 \
  accel_deadband_mps2:=0.15 \
  accel_velocity_leak_tau_sec:=1.0
```

## Test commands

Slow forward latched command. Because latching is enabled by default, one publish is enough. Expected RPM sign pattern is `[+, +, +, -, -, -]`:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Explicit stop command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Slow in-place positive yaw command. With the default mapping, this makes the right side walk forward and the left side walk backward:

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.20}}"
```

Monitor the outgoing motor RPM array:

```bash
ros2 topic echo /motor_rpm_cmd
```

Monitor command propagation fields from the ESP bridge:

```bash
ros2 topic echo /motor_output_odom
```

## Main tuneable launch arguments

`linear_rpm_per_mps`, `yaw_rpm_per_radps`, `max_motor_rpm`, `phase_kp_rpm_per_rad`, `max_phase_correction_rpm`, `yaw_rate_feedback_kp`, `linear_velocity_feedback_kp`, `max_linear_velocity_correction_mps`, `tilt_safety_enabled`, `imu_feedback_enabled`, and `euler_feedback_enabled` are exposed directly as launch arguments.

All six-element arrays in the YAML use `[right_back, right_middle, right_front, left_front, left_middle, left_back]` order.
