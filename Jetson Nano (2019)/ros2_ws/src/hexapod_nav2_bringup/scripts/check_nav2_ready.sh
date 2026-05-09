#!/usr/bin/env bash
set -euo pipefail

printf '\nRequired topic rates:\n'
ros2 topic hz /scan --window 10 || true
ros2 topic hz /odom --window 10 || true

printf '\nOne-shot topic checks:\n'
ros2 topic echo /map --once || true
ros2 topic echo /cmd_vel --once || true

printf '\nTF chain checks:\n'
ros2 run tf2_ros tf2_echo map base_link --once || true
ros2 run tf2_ros tf2_echo odom base_link --once || true
ros2 run tf2_ros tf2_echo base_link orbslam_scan_frame --once || true

printf '\nLifecycle states:\n'
for node in controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother smoother_server; do
  ros2 lifecycle get "/${node}" || true
done
