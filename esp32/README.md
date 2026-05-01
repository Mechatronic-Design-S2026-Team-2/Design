# DSY-RS ROS 2 velocity control bundle - micro-ROS over UART0 custom transport

This bundle targets the current working single-owner RS485 version and changes only the micro-ROS transport layer:

- `host_ws/src/dsy_motor_msgs`: custom ROS 2 messages.
- `host_ws/src/dsy_motor_gui`: simple ROS 2 Tkinter slider GUI.
- `esp32_main`: ESP-IDF `main/` replacement files using micro-ROS over UART0 custom transport.
- `microros_extra_packages/dsy_motor_msgs`: copy of the custom interface package for `micro_ros_espidf_component/extra_packages`.
- `serial_transport_colcon_meta_snippet.json`: the key transport change you must merge into the component `colcon.meta`.

## Resulting port usage

- RS485 motor bus stays on UART2: TX=17, RX=16, DE/RE=4.
- micro-ROS uses UART0 through the ESP32 board USB-UART bridge: TX0/RX0.
- GPIO18 remains the servo-enable output for the external optocoupled interface.

## Host ROS 2 workspace

```bash
mkdir -p ~/dsy_ros_ws/src
cp -r host_ws/src/dsy_motor_msgs ~/dsy_ros_ws/src/
cp -r host_ws/src/dsy_motor_gui ~/dsy_ros_ws/src/
cd ~/dsy_ros_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

Run the GUI:

```bash
ros2 launch dsy_motor_gui dsy_motor_slider_gui.launch.py
```

## ESP-IDF project setup

Assume your ESP-IDF project root is `/path/to/servo_bridge`.

### 1. Clone the micro-ROS ESP-IDF component

```bash
cd /path/to/servo_bridge
mkdir -p components
cd components
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
```

### 2. Install build dependencies in the IDF Python environment

Use an IDF shell, not a shell with ROS sourced.

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
```

### 3. Copy the UART transport bridge files into your project

```bash
cd /path/to/servo_bridge
mkdir -p main
cp -r /path/to/this_bundle/esp32_main/* main/
```

### 4. Add the custom interface package to the micro-ROS component

```bash
cp -r /path/to/this_bundle/microros_extra_packages/dsy_motor_msgs \
  /path/to/servo_bridge/components/micro_ros_espidf_component/extra_packages/
```

### 5. Enable custom transport in the component `colcon.meta`

Edit:

```text
/path/to/servo_bridge/components/micro_ros_espidf_component/colcon.meta
```

and add:

```json
"-DRMW_UXRCE_TRANSPORT=custom"
```

to the `rmw_microxrcedds` `cmake-args` list.

A snippet is included in `serial_transport_colcon_meta_snippet.json`.

### 6. Clean and rebuild micro-ROS libraries

```bash
cd /path/to/servo_bridge
idf.py clean-microros
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py flash
```

## ESP `menuconfig`

Set the project values under the custom `DSY RS485 Bench-Safe Control` menu:

- `SERVO_RS485_UART_TXD = 17`
- `SERVO_RS485_UART_RXD = 16`
- `SERVO_RS485_UART_DE_RE = 4`
- `SERVO_RS485_BAUD_RATE = 38400`
- `MICRO_ROS_UART_BAUD_RATE = 460800`
- `SERVO_SAFE_MAX_MOTOR_RPM = 120`
- `SERVO_SAFE_TORQUE_LIMIT_TENTHS_PERCENT_RATED = 200`
- `SERVO_TEST_ACTIVE_DRIVE_COUNT = 1` for first bring-up, then raise as needed.

Do not run `idf.py monitor` at the same time as the micro-ROS serial agent, because both use the same USB-UART path on UART0.

## micro-ROS Agent on the ROS 2 Jazzy laptop

### Preferred: source build on the laptop

```bash
mkdir -p ~/uros_agent_ws/src
cd ~/uros_agent_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/uros_agent_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Run the agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 460800 -v6
```

Replace `/dev/ttyUSB0` with your actual adapter node.

### Docker fallback

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:kilted serial --dev /dev/ttyUSB0 -b 460800 -v6
```

## Suggested bring-up order

1. Flash the ESP32.
2. Disconnect `idf.py monitor` if it is open.
3. Start the serial micro-ROS agent on the laptop.
4. Power the drive and ESP32.
5. Source `~/dsy_ros_ws/install/setup.bash`.
6. Run `ros2 topic list` and confirm the `/dsy_motor_N/*` topics appear.
7. Start the GUI and command very small velocities first.

## Safety defaults preserved

- Speed mode is still used for direct velocity control.
- Torque is capped at `200` DSY units (`20.0% rated`) to stay conservative on a 30 V / 3 A bench supply without external brake resistors.
- Maximum motor speed stays at `120 rpm`.
- Useful odometry is the 1:50 gearbox output shaft; the bridge converts motor-side feedback before publishing.
