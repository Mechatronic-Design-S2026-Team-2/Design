# hexapod_orbslam_udp_bridge

ROS 2 Jazzy C++ UDP bridge for the bare-metal ORB-SLAM2 RealSense process.

It listens for ORB-SLAM UDP packets and publishes:

- `/odom` as `nav_msgs/msg/Odometry`
- optional TF `odom -> base_link`
- `/scan` as `sensor_msgs/msg/LaserScan` for `slam_toolbox`
- `/orbslam/map_points` as `sensor_msgs/msg/PointCloud2` for RViz/debug

The sparse ORB-SLAM map-point cloud is not the occupancy map. The occupancy map is produced by `slam_toolbox` from `/scan`.

## Patch notes for slam_toolbox

This version changes the bridge defaults for occupancy mapping:

- Enables virtual-scan support by default.
- Publishes scan in `orbslam_scan_frame`, with a static `base_link -> orbslam_scan_frame` transform.
- Converts the virtual scan from camera optical convention into ROS laser convention.
- Reverses the scan range order so left/right are correct in ROS.
- Stamps scans with the latest pose stamp to avoid `slam_toolbox` TF extrapolation failures.
- Defaults `base_frame` to `base_link`.
- Defaults camera offset for an 11 inch forward D415: `camopt_to_base_z = -0.2794`, `scan_x = 0.2794`.

## Frame convention

ORB-SLAM/OpenCV camera optical axes:

```text
+X = right
+Y = down
+Z = forward
```

ROS body/laser axes:

```text
+X = forward
+Y = left
+Z = up
```

For map/pose conversion, keep:

```yaml
convert_orbslam_to_ros_axes: true
```

For scan conversion, keep:

```yaml
convert_scan_optical_to_ros: true
```

## Camera offset

The bridge uses two offset conventions.

`camopt_to_base_{x,y,z}` is the vector from the camera optical origin to `base_link`, expressed in camera optical axes. For a camera 0.2794 m forward of `base_link`:

```text
camopt_to_base_x = 0.0
camopt_to_base_y = 0.0
camopt_to_base_z = -0.2794
```

The scan static transform is expressed in `base_link` axes. For the same 0.2794 m forward camera:

```text
scan_x = 0.2794
scan_y = 0.0
scan_z = 0.0
```

Set the real camera height using `scan_z` and the corresponding camera optical offset using `camopt_to_base_y = camera_height_above_base`.

## Build in the container

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select hexapod_orbslam_udp_bridge
source install/setup.bash
```

## Install patched systemd service on bare metal

Copy the service from this package or from the zip's `systemd/` directory:

```bash
sudo cp /ros2_ws/src/hexapod_orbslam_udp_bridge/systemd/orbslam2-realsense.service /etc/systemd/system/orbslam2-realsense.service
sudo systemctl daemon-reload
sudo systemctl restart orbslam2-realsense.service
sudo journalctl -u orbslam2-realsense.service -f
```

Important service fixes:

- The sender uses `ORBSLAM_UDP_IP`, not `ORBSLAM_UDP_HOST`.
- `ORBSLAM_UDP_SCAN_ENABLE=1` must remain enabled.
- The ROS 2 container should use `--net=host` if the service sends to `127.0.0.1`.

## Run bridge only

```bash
ros2 launch hexapod_orbslam_udp_bridge udp_bridge.launch.py
```

For your known 11 inch forward camera offset, the defaults are already:

```bash
camopt_to_base_z:=-0.2794
scan_x:=0.2794
```

If the camera is also 0.25 m above `base_link`, launch with:

```bash
ros2 launch hexapod_orbslam_udp_bridge udp_bridge.launch.py \
  camopt_to_base_y:=0.25 \
  scan_z:=0.25
```

## Run bridge plus slam_toolbox

```bash
ros2 launch hexapod_orbslam_udp_bridge orbslam_slam_toolbox.launch.py
```

This uses:

```text
config/slam_toolbox_orbslam_scan.yaml
```

The expected TF tree is:

```text
map -> odom -> base_link -> orbslam_scan_frame
```

`slam_toolbox` publishes `map -> odom`; the bridge publishes `odom -> base_link`; the launch file publishes the static `base_link -> orbslam_scan_frame`.

## Debug

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
ros2 topic echo /map --once
```

If `/scan` is missing, check the bare-metal service environment first:

```bash
sudo systemctl show orbslam2-realsense.service -p Environment
sudo journalctl -u orbslam2-realsense.service -n 100
```

If UDP packets are not reaching the container and the service sends to `127.0.0.1`, recreate the ROS 2 container with host networking.
