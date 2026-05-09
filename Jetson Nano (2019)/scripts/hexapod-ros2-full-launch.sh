#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${HEXAPOD_CONTAINER_NAME:-jazzy}"
MICROROS_AGENT_CONTAINER="${HEXAPOD_MICROROS_AGENT_CONTAINER:-microros_agent}"
WAIT_FOR_MICROROS_AGENT="${HEXAPOD_WAIT_FOR_MICROROS_AGENT:-true}"
ROS_SETUP="${HEXAPOD_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${HEXAPOD_WS_SETUP:-/ros2_ws/install/setup.bash}"
MAP_DIR="${HEXAPOD_MAP_DIR:-/ros2_ws/maps}"
OPERATOR_URL="${HEXAPOD_OPERATOR_URL:-http://192.168.50.1:8080/}"
OPERATOR_HTTP_PORT="${HEXAPOD_OPERATOR_HTTP_PORT:-8080}"
LCD_I2C_DEVICE="${HEXAPOD_LCD_I2C_DEVICE:-/dev/i2c-1}"
LCD_I2C_ADDRESS="${HEXAPOD_LCD_I2C_ADDRESS:-39}"
LCD_STATIC_URL_ONLY="${HEXAPOD_LCD_STATIC_URL_ONLY:-true}"
NAV2_START_DELAY_SEC="${HEXAPOD_NAV2_START_DELAY_SEC:-10.0}"
START_ORBSLAM_SLAMTOOLBOX="${HEXAPOD_START_ORBSLAM_SLAMTOOLBOX:-true}"
START_HARDWARE_STACK="${HEXAPOD_START_HARDWARE_STACK:-true}"
START_TRIPOD_CONTROLLER="${HEXAPOD_START_TRIPOD_CONTROLLER:-true}"
START_OPERATOR_INTERFACE="${HEXAPOD_START_OPERATOR_INTERFACE:-true}"
START_LCD_STATUS="${HEXAPOD_START_LCD_STATUS:-true}"
START_IMU_UDP_EXPORT="${HEXAPOD_START_IMU_UDP_EXPORT:-false}"
START_MERGED_VISUALIZATION="${HEXAPOD_START_MERGED_VISUALIZATION:-false}"
OPERATOR_DEFAULT_MODE="${HEXAPOD_OPERATOR_DEFAULT_MODE:-estop}"
OPERATOR_LATCH_TELEOP_COMMANDS="${HEXAPOD_OPERATOR_LATCH_TELEOP_COMMANDS:-true}"
OPERATOR_KEYBOARD_LINEAR_MPS="${HEXAPOD_OPERATOR_KEYBOARD_LINEAR_MPS:-0.50}"
OPERATOR_KEYBOARD_ANGULAR_RADPS="${HEXAPOD_OPERATOR_KEYBOARD_ANGULAR_RADPS:-0.50}"
OPERATOR_MAX_LINEAR_MPS="${HEXAPOD_OPERATOR_MAX_LINEAR_MPS:-3.00}"
OPERATOR_MAX_ANGULAR_RADPS="${HEXAPOD_OPERATOR_MAX_ANGULAR_RADPS:-3.00}"
OPERATOR_WAYPOINT_MAX_LINEAR_MPS="${HEXAPOD_OPERATOR_WAYPOINT_MAX_LINEAR_MPS:-0.50}"
OPERATOR_WAYPOINT_MAX_ANGULAR_RADPS="${HEXAPOD_OPERATOR_WAYPOINT_MAX_ANGULAR_RADPS:-0.50}"
OPERATOR_HOST_CONTROL_BASE_URL="${HEXAPOD_OPERATOR_HOST_CONTROL_BASE_URL:-http://127.0.0.1:18080}"
OPERATOR_MAPPING_UPDATES_ENABLED_DEFAULT="${HEXAPOD_OPERATOR_MAPPING_UPDATES_ENABLED_DEFAULT:-true}"
ORBSLAM_SCAN_PUBLISH_DECIMATION="${HEXAPOD_ORBSLAM_SCAN_PUBLISH_DECIMATION:-3}"
CLEAN_STALE_ROS="${HEXAPOD_CLEAN_STALE_ROS:-true}"
DISABLE_FASTDDS_SHM="${HEXAPOD_DISABLE_FASTDDS_SHM:-true}"

until docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null | grep -q true; do
  sleep 1
done

if [ "${WAIT_FOR_MICROROS_AGENT}" = "true" ]; then
  for _ in $(seq 1 30); do
    if docker inspect -f '{{.State.Running}}' "${MICROROS_AGENT_CONTAINER}" 2>/dev/null | grep -q true; then
      break
    fi
    sleep 1
  done
fi

# Do cleanup in a short, separate docker exec before the long-running launch.
# The previous script did broad pkill -f cleanup inside the same bash -lc process
# used for the long-running launch. That could match and kill itself, producing
# systemd status=143 restart loops.
if [ "${CLEAN_STALE_ROS}" = "true" ]; then
  docker exec "${CONTAINER_NAME}" bash -lc '
    set +e
    pkill -INT -f "[r]os2 launch hexapod_nav2_bringup" 2>/dev/null || true
    pkill -INT -f "[o]perator_interface_node|[h]exapod_operator_interface" 2>/dev/null || true
    pkill -INT -x tripod_gait_node 2>/dev/null || true
    pkill -INT -x udp_bridge 2>/dev/null || true
    pkill -INT -x async_slam_toolbox_node 2>/dev/null || true
    pkill -INT -x controller_server 2>/dev/null || true
    pkill -INT -x smoother_server 2>/dev/null || true
    pkill -INT -x planner_server 2>/dev/null || true
    pkill -INT -x behavior_server 2>/dev/null || true
    pkill -INT -x bt_navigator 2>/dev/null || true
    pkill -INT -x waypoint_follower 2>/dev/null || true
    pkill -INT -x velocity_smoother 2>/dev/null || true
    pkill -INT -x lifecycle_manager 2>/dev/null || true
    sleep 2
    pkill -TERM -f "[o]perator_interface_node|[h]exapod_operator_interface" 2>/dev/null || true
    pkill -TERM -x tripod_gait_node 2>/dev/null || true
    pkill -TERM -x udp_bridge 2>/dev/null || true
    pkill -TERM -x async_slam_toolbox_node 2>/dev/null || true
    pkill -TERM -x controller_server 2>/dev/null || true
    pkill -TERM -x smoother_server 2>/dev/null || true
    pkill -TERM -x planner_server 2>/dev/null || true
    pkill -TERM -x behavior_server 2>/dev/null || true
    pkill -TERM -x bt_navigator 2>/dev/null || true
    pkill -TERM -x waypoint_follower 2>/dev/null || true
    pkill -TERM -x velocity_smoother 2>/dev/null || true
    pkill -TERM -x lifecycle_manager 2>/dev/null || true
    sleep 1
  ' || true
fi

if [ "${DISABLE_FASTDDS_SHM}" = "true" ]; then
  docker exec "${CONTAINER_NAME}" bash -lc 'rm -f /dev/shm/fastrtps_port* 2>/dev/null || true' || true
fi

exec docker exec \
  -e FASTDDS_BUILTIN_TRANSPORTS="${DISABLE_FASTDDS_SHM:+UDPv4}" \
  "${CONTAINER_NAME}" bash -lc "
  set -e
  if [ '${DISABLE_FASTDDS_SHM}' = 'true' ]; then
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  fi
  source '${ROS_SETUP}'
  source '${WS_SETUP}'
  mkdir -p '${MAP_DIR}'

  exec ros2 launch hexapod_nav2_bringup hexapod_nav2.launch.py \
    start_orbslam_slamtoolbox:='${START_ORBSLAM_SLAMTOOLBOX}' \
    start_hardware_stack:='${START_HARDWARE_STACK}' \
    start_tripod_controller:='${START_TRIPOD_CONTROLLER}' \
    start_operator_interface:='${START_OPERATOR_INTERFACE}' \
    start_lcd_status:='${START_LCD_STATUS}' \
    start_imu_udp_export:='${START_IMU_UDP_EXPORT}' \
    start_merged_visualization:='${START_MERGED_VISUALIZATION}' \
    operator_http_port:='${OPERATOR_HTTP_PORT}' \
    operator_url:='${OPERATOR_URL}' \
    lcd_i2c_device:='${LCD_I2C_DEVICE}' \
    lcd_i2c_address:='${LCD_I2C_ADDRESS}' \
    lcd_operator_url:='${OPERATOR_URL}' \
    lcd_static_url_only:='${LCD_STATIC_URL_ONLY}' \
    operator_map_storage_dir:='${MAP_DIR}' \
    operator_default_mode:='${OPERATOR_DEFAULT_MODE}' \
    operator_latch_teleop_commands:='${OPERATOR_LATCH_TELEOP_COMMANDS}' \
    operator_keyboard_linear_mps:='${OPERATOR_KEYBOARD_LINEAR_MPS}' \
    operator_keyboard_angular_radps:='${OPERATOR_KEYBOARD_ANGULAR_RADPS}' \
    operator_max_linear_mps:='${OPERATOR_MAX_LINEAR_MPS}' \
    operator_max_angular_radps:='${OPERATOR_MAX_ANGULAR_RADPS}' \
    operator_waypoint_max_linear_mps:='${OPERATOR_WAYPOINT_MAX_LINEAR_MPS}' \
    operator_waypoint_max_angular_radps:='${OPERATOR_WAYPOINT_MAX_ANGULAR_RADPS}' \
    operator_host_control_base_url:='${OPERATOR_HOST_CONTROL_BASE_URL}' \
    operator_mapping_updates_enabled_default:='${OPERATOR_MAPPING_UPDATES_ENABLED_DEFAULT}' \
    orbslam_scan_publish_decimation:='${ORBSLAM_SCAN_PUBLISH_DECIMATION}' \
    nav2_start_delay_sec:='${NAV2_START_DELAY_SEC}'
"
