# klann-esp32

ESP-IDF firmware for the ESP32 side of the Klann-linkage robot control stack.

This application currently does four main things:
- reads six force-sensitive resistors with the ESP32 ADC continuous driver
- reads an MPU-9250 / MPU-6500-compatible IMU over I2C
- publishes IMU and force-sensor telemetry over micro-ROS serial transport
- shows compact runtime and sensor status on a 16x2 HD44780-compatible LCD

CAN-related motor control is still placeholder-only in this firmware. The micro-ROS task caches `cmd_vel`, but no actuator-side CAN command output is implemented yet.

## Project layout

```text
klann-esp32/
├── CMakeLists.txt
├── Kconfig.projbuild
├── README.md
├── main/
│   ├── main.c
│   ├── esp32_serial_transport.c
│   ├── esp32_serial_transport.h
│   ├── force_sensor_adc.c
│   ├── force_sensor_adc.h
│   ├── mpu9250.c
│   ├── mpu9250.h
│   ├── character_lcd.c
│   └── character_lcd.h
└── extra_ros_packages/
    └── klann_msgs/
        ├── CMakeLists.txt
        ├── package.xml
        └── msg/
            └── ForceSensorVoltages.msg
```

## Implemented interfaces

### Force sensors
- 6 sensors total
- GPIO36, GPIO39, GPIO34, GPIO35, GPIO32, GPIO33
- continuous ADC sampling with cached raw counts and millivolts
- published as `klann_msgs/msg/ForceSensorVoltages` on `force_sensor_voltage_millivolts`

### IMU
- I2C port: `I2C_NUM_0`
- SCL: GPIO21
- SDA: GPIO22
- INT: GPIO19
- supported `WHO_AM_I` values:
  - `0x71` -> MPU-9250
  - `0x70` -> MPU-6500-compatible accel/gyro-only path
- published as `sensor_msgs/msg/Imu` on `imu/data_raw`

### LCD
- HD44780-compatible 16x2 LCD in 4-bit mode
- RS: GPIO25
- E: GPIO26
- D4: GPIO18
- D5: GPIO27
- D6: GPIO14
- D7: GPIO13
- `RW` should be tied to GND in hardware
- `VO` should be driven by a contrast pot, not a fixed resistor to GND

### micro-ROS serial transport
- transport backend uses the ESP-IDF UART driver
- current application binds transport to `UART_NUM_0`
- intended host link is the ESP32 dev board USB-UART bridge
- firmware UART transport baud rate: `115200`

## LCD output

The LCD status task refreshes every 250 ms.

Line 1 format:
- `u<agent> e<entities> p<count>`
- `u1` means the micro-ROS agent is reachable
- `e1` means node / publishers / subscriber / timer / executor are initialized
- `p<count>` is the publish cycle count modulo 10000

Line 2 format:
- `F<index> <voltage>mV rc<code>`
- rotates force-sensor index once per second
- shows the cached millivolt reading for the currently selected sensor
- `rc<code>` is the cached last rcl / rmw error code modulo 100

## Before building

Assumptions:
- ESP-IDF environment is already installed and sourced
- the `micro_ros_espidf_component` directory is already present under `components/`
- this `klann-esp32` directory is the ESP-IDF project root
- the custom message package shown above exists under `extra_ros_packages/`

Typical shell setup, if needed:

```bash
. /path/to/esp-idf/export.sh
```

## Required menuconfig checks

Run:

```bash
idf.py menuconfig
```

Check these settings before building:

### micro-ROS task sizing
From `Kconfig.projbuild` in this project:
- `MICRO_ROS_APP_STACK`
- `MICRO_ROS_APP_TASK_PRIO`

The defaults in this project are usually fine unless you add more publishers, subscriptions, or larger messages.

### micro-ROS UART transport pins
The serial transport source uses these Kconfig symbols:
- `CONFIG_MICROROS_UART_TXD`
- `CONFIG_MICROROS_UART_RXD`
- `CONFIG_MICROROS_UART_RTS`
- `CONFIG_MICROROS_UART_CTS`

For a typical ESP32 dev board using the onboard USB-UART bridge and `UART0`, use:
- TXD = GPIO1
- RXD = GPIO3
- RTS = no connection / unused
- CTS = no connection / unused

## Build steps

From the `klann-esp32` directory:

```bash
idf.py set-target esp32
idf.py fullclean
rm -rf components/micro_ros_espidf_component/include
rm -rf components/micro_ros_espidf_component/micro_ros_src
rm -f components/micro_ros_espidf_component/libmicroros.a
idf.py build
```

Notes:
- `set-target` is only needed the first time, or if the target changed.
- `fullclean` plus removal of the generated micro-ROS include / library directories forces a fresh interface rebuild, which is useful after changing custom messages.

## Flash steps

Find the board serial device first:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Then flash:

```bash
idf.py -p /dev/ttyUSB0 flash
```

## Serial monitor

After flashing:

```bash
idf.py -p /dev/ttyUSB0 monitor
```

monitor controls:
- `Ctrl+]` exits monitor
- `Ctrl+T`, then `Ctrl+R` resets the board

## Host-side micro-ROS agent setup

If `ros2 run micro_ros_agent micro_ros_agent ...` already works on the host PC, you can skip this section.

If the agent is not installed yet, set it up in a normal ROS 2 workspace. The official micro-ROS workflow uses the `micro_ros_setup` package to create and build the agent workspace, then sources the resulting installation before running the agent. citeturn2search2turn1view1turn1view2

Example host-side setup:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/microros_agent_ws/src
cd ~/microros_agent_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

The micro-ROS Agent supports serial transport, which is the transport used by this ESP32 firmware over the dev board USB-UART bridge.

## Host-side micro-ROS agent

On the host PC, start the serial agent at the same baud rate used in `esp32_serial_transport.c`:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

## Host-side custom message package

Because the ESP32 publishes a custom message type, the host ROS 2 environment also needs the same `klann_msgs` package built in a normal ROS 2 workspace.

Example:

```bash
mkdir -p ~/klann_msgs_ws/src
cp -r extra_ros_packages/klann_msgs ~/klann_msgs_ws/src/
cd ~/klann_msgs_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Use that sourced shell for `ros2 topic echo`.

## Quick verification commands

Send a placeholder command input:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```
