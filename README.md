# Hexapod Operation README

## Overview

This document describes basic setup, operation, charging, connection, and troubleshooting procedures for the hexapod robot.

---

## Aligning Motor Cranks

For the robot to maintain the expected alternating tripod gait, each input crank — the first bar extending from each motor — should point straight down.

This crank position corresponds to the highest point in the leg movement. If the cranks are not aligned this way, the control computer may incorrectly estimate leg position and phase. The robot may still operate, but gait smoothness may be reduced.

---

## Emergency Disable Switch

The switch mounted on the top surface of the robot disables motor control and the power relays.

| Switch Position | Function |
|---|---|
| `O` | Normal operation |
| `I` | Reset / disabled state |

To reset the motor control system, move the switch to `I` for a few moments, then return it to `O`.

---

## Charging the Robot Battery

1. Turn on the emergency disable switch by moving it to the `I` position.
2. Disconnect the XT90 connector from the board.
3. The battery may either be removed or charged in place.
4. Connect the charger to the smaller XT60 connector on the battery.
5. After the XT60 connector is attached, plug the charger power cord into a standard AC outlet.
6. Both charger LEDs should turn red.
7. When charging is complete, one charger LED will turn green.
8. Disconnect the charger from AC power first.
9. Disconnect the XT60 connector between the charger and the battery.

---

## Connecting to the Robot

### Wi-Fi

The robot hosts a Wi-Fi network with the following credentials:

| Field | Value |
|---|---|
| SSID | `team2` |
| Password | `MECHATRONICS` |
| Robot IP | `192.168.50.1` |

### Ethernet

When connected through the Jetson Ethernet port, the robot IP is:

```text
192.168.55.2
```

---

## SSH Access

The robot operating system credentials are:

| Field | Value |
|---|---|
| Username | `team2` |
| Password | `MECHATRONICS` |

Example SSH command over Wi-Fi:

```bash
ssh team2@192.168.50.1
```

Example SSH command over Ethernet:

```bash
ssh team2@192.168.55.2
```

---

## Accessing the Robot Control Interface

The robot control interface is available in a web browser.

### Over Wi-Fi

```text
http://192.168.50.1:8080
```

### Over Ethernet

```text
http://192.168.55.2:8080
```

Any modern desktop or laptop web browser should work.

When the control interface tab is active, pressing the displayed keyboard keys will command robot movement. The main map display can be used to view robot movement, the current occupancy grid map, generated maps, and waypoint placement.

Map saving and loading controls are located near the bottom of the right sidebar, along with key bindings, status information, and maximum velocity settings.

---

## Troubleshooting Wi-Fi

Under normal conditions, if the battery is charged, the Wi-Fi network should appear a few seconds after the battery is connected.

If the Wi-Fi network does not appear:

1. Check normal client-device Wi-Fi issues first.
2. Inspect the Jetson Nano located at the back-left corner inside the chassis.
3. Confirm that the green LED on the left side of the Jetson is on.
4. If the Jetson is not powered, check that the power jack connection is secure.

---

## Troubleshooting the Control Interface

If the robot and control device are connected, but the web control interface does not load:

1. Verify that the URL matches the connection method:
   - Wi-Fi: `http://192.168.50.1:8080`
   - Ethernet: `http://192.168.55.2:8080`
2. Open an SSH session to the robot.
3. Run the following commands:

```bash
docker exec -it jazzy bash
sudo systemctl restart orbslam2-realsense.service
sudo systemctl restart hexapod-ros2-full-launch.service
```

---

## Troubleshooting Robot Motion

If the robot does not move its legs when a nonzero motion command is sent, the most likely issue is the emergency disable state.

1. Move the emergency disable switch to `I`.
2. Wait a few moments.
3. Move the switch back to `O`.

The red PCB module on the right side of the robot has three LEDs. The two smaller LEDs should flash initially, then remain on.

If the LEDs do not remain on:

1. Disconnect and reconnect the USB-C cable connected to the ESP32.
2. Check the USB-A side of the cable connected to the Jetson.
3. Repeat the emergency switch reset procedure.

If the robot still does not move, restart the micro-ROS and ROS containers:

```bash
docker restart microros_agent
docker restart jazzy
```

---

## Foot Adjustment

The feet are height-adjustable relative to the leg assemblies. Before operation, verify that the feet make proper contact with the ground.

The legs may be moved manually when the robot is unpowered, or the robot may be walked using the standard control interface.

Adjustment guidance:

| Condition | Adjustment |
|---|---|
| Feet slip while walking | Rotate the threaded bolt holding the rubber foot pad counterclockwise |
| Feet extend too far and support wheels lose ground contact | Rotate the threaded bolt clockwise |

---

## Caster Wheel Adjustment

The caster wheels at each corner of the robot are also mounted using threaded bolts.

If the caster wheels have inconsistent heights, rotate the threaded bolts to adjust their height until the chassis is properly supported.

---

## Battery BMS Access

The battery includes a Bluetooth-enabled Daly BMS. BMS settings related to current output and protection can be accessed while the battery is actively charging or discharging.

| Field | Value |
|---|---|
| Bluetooth ID | `52V 25ah` |
| App | Daly BMS app |
| Password | `123456` |

To access the BMS:

1. Download the Daly BMS app.
2. Ensure the battery is actively charging or discharging.
3. Search for the Bluetooth ID `52V 25ah`.
4. Connect using password `123456`.
