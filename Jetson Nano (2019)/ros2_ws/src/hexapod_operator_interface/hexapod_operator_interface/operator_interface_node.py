#!/usr/bin/env python3
"""HTTP operator interface, cmd_vel arbiter, map store, and web UI."""

from __future__ import annotations

import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty

from dsy_motor_msgs.msg import MotorOutputOdometryArray
from hexapod_control_interfaces.msg import HexapodMotorState

MODE_ESTOP = 'estop'
MODE_TELEOP = 'teleop'
MODE_WAYPOINT = 'waypoint'
VALID_MODES = {MODE_ESTOP, MODE_TELEOP, MODE_WAYPOINT}


class HexapodOperatorInterface(Node):
    """Arbitrate Nav2, teleop, and estop while hosting a simple browser UI."""

    def __init__(self) -> None:
        super().__init__('hexapod_operator_interface')
        self._lock = threading.RLock()

        self.bind_address = self.declare_parameter('bind_address', '0.0.0.0').value
        self.http_port = int(self.declare_parameter('http_port', 8080).value)
        self.selected_cmd_vel_topic = self.declare_parameter(
            'selected_cmd_vel_topic', '/hexapod/cmd_vel_selected').value
        self.nav_cmd_vel_topic = self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel_nav').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.global_costmap_topic = self.declare_parameter(
            'global_costmap_topic', '/global_costmap/costmap').value
        self.local_costmap_topic = self.declare_parameter(
            'local_costmap_topic', '/local_costmap/costmap').value
        self.map_fallback_to_costmaps = bool(self.declare_parameter('map_fallback_to_costmaps', True).value)
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.compact_odom_topic = self.declare_parameter('compact_odom_topic', '/motor_output_odom').value
        self.motor_state_topic = self.declare_parameter('motor_state_topic', '/hexapod/motor_state').value
        self.loaded_map_topic = self.declare_parameter(
            'loaded_map_topic', '/hexapod/operator/loaded_map').value
        self.map_storage_dir = Path(str(self.declare_parameter('map_storage_dir', '/ros2_ws/maps').value))
        self.map_occupied_thresh = float(self.declare_parameter('map_occupied_thresh', 0.65).value)
        self.map_free_thresh = float(self.declare_parameter('map_free_thresh', 0.25).value)

        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 20.0).value)
        self.teleop_timeout_sec = float(self.declare_parameter('teleop_timeout_sec', 0.35).value)
        self.latch_teleop_commands = bool(self.declare_parameter('latch_teleop_commands', True).value)
        self.nav_cmd_timeout_sec = float(self.declare_parameter('nav_cmd_timeout_sec', 0.50).value)
        self.keyboard_linear_mps = float(self.declare_parameter('keyboard_linear_mps', 0.50).value)
        self.keyboard_angular_radps = float(self.declare_parameter('keyboard_angular_radps', 0.50).value)
        self.keyboard_fast_scale = float(self.declare_parameter('keyboard_fast_scale', 1.6).value)
        self.keyboard_slow_scale = float(self.declare_parameter('keyboard_slow_scale', 0.5).value)
        self.max_linear_mps = abs(float(self.declare_parameter('max_linear_mps', 3.00).value))
        self.max_angular_radps = abs(float(self.declare_parameter('max_angular_radps', 3.00).value))
        self.waypoint_max_linear_mps = abs(float(
            self.declare_parameter('waypoint_max_linear_mps', 0.50).value))
        self.waypoint_max_angular_radps = abs(float(
            self.declare_parameter('waypoint_max_angular_radps', 0.50).value))
        self.robot_body_length_m = abs(float(self.declare_parameter('robot_body_length_m', 0.76).value))
        self.robot_body_width_m = abs(float(self.declare_parameter('robot_body_width_m', 1.10).value))
        self.robot_stride_visual_m = abs(float(self.declare_parameter('robot_stride_visual_m', 0.18).value))
        self.robot_leg_lateral_visual_m = abs(float(
            self.declare_parameter('robot_leg_lateral_visual_m', 0.18).value))
        self.publish_status_json = bool(self.declare_parameter('publish_status_json', True).value)
        self.host_control_base_url = str(
            self.declare_parameter('host_control_base_url', 'http://127.0.0.1:18080').value).rstrip('/')
        self.host_control_timeout_sec = float(
            self.declare_parameter('host_control_timeout_sec', 8.0).value)
        self.pause_mapping_service = str(
            self.declare_parameter('pause_mapping_service', '/slam_toolbox/pause_new_measurements').value)
        self.resume_mapping_service = str(
            self.declare_parameter('resume_mapping_service', '/slam_toolbox/resume_new_measurements').value)
        self.mapping_updates_enabled_default = bool(
            self.declare_parameter('mapping_updates_enabled_default', True).value)
        self.mapping_enable_topic = str(
            self.declare_parameter('mapping_enable_topic', '/hexapod/operator/map_updates_enabled').value)

        default_mode = str(self.declare_parameter('default_mode', MODE_ESTOP).value).lower()
        if default_mode not in VALID_MODES:
            self.get_logger().warn(f"invalid default_mode '{default_mode}', using estop")
            default_mode = MODE_ESTOP

        self._mode = default_mode
        self._teleop_cmd = Twist()
        self._last_teleop_time = 0.0
        self._last_nav_cmd = Twist()
        self._last_nav_time = 0.0
        self._latest_battery_v: Optional[float] = None
        self._latest_motor_phase_rad = [0.0] * 6
        self._have_motor_phase = False
        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_map_source = ''
        self._last_map_time = 0.0
        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._last_selected_cmd = Twist()
        self._last_mode_change_time = time.monotonic()
        self._last_goal_status = 'idle'
        self._last_system_action_status = 'idle'
        self._mapping_updates_enabled = self.mapping_updates_enabled_default
        self._goal_handle = None

        self._cmd_pub = self.create_publisher(Twist, self.selected_cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, '/hexapod/operator/status_json', 10)
        self._battery_pub = self.create_publisher(Float32, '/hexapod/operator/battery_voltage', 10)
        mapping_enable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mapping_enable_pub = self.create_publisher(Bool, self.mapping_enable_topic, mapping_enable_qos)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._loaded_map_pub = self.create_publisher(OccupancyGrid, self.loaded_map_topic, map_qos)
        self.map_storage_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(Twist, self.nav_cmd_vel_topic, self._nav_cmd_callback, 10)
        self._map_subscriptions = []
        self._create_map_subscriptions()
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

        compact_odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            MotorOutputOdometryArray,
            self.compact_odom_topic,
            self._compact_odom_callback,
            compact_odom_qos)
        self.create_subscription(HexapodMotorState, self.motor_state_topic, self._motor_state_callback, 10)

        self._pause_mapping_client = self.create_client(Empty, self.pause_mapping_service)
        self._resume_mapping_client = self.create_client(Empty, self.resume_mapping_service)
        self._navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publish the initial mapping state for scan_gate_node. Do not depend on
        # slam_toolbox pause/resume services, since they are not available on all builds.
        self._publish_mapping_enable(self.mapping_updates_enabled_default)

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.create_timer(period, self._publish_selected_cmd_timer)
        self.create_timer(0.25, self._publish_status_timer)

        self._http_server = self._make_http_server()
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f"operator interface ready: http://{self.bind_address}:{self.http_port}/ "
            f"mode={self._mode} nav_cmd={self.nav_cmd_vel_topic} selected_cmd={self.selected_cmd_vel_topic} "
            f"map_dir={self.map_storage_dir}")

    def destroy_node(self) -> bool:
        try:
            self._http_server.shutdown()
            self._http_server.server_close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()

    def _make_http_server(self) -> ThreadingHTTPServer:
        parent = self

        class Handler(BaseHTTPRequestHandler):
            server_version = 'HexapodOperatorHTTP/0.2'

            def log_message(self, fmt: str, *args: Any) -> None:
                parent.get_logger().debug(fmt % args)

            def _read_json(self) -> Dict[str, Any]:
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0:
                    return {}
                raw = self.rfile.read(length).decode('utf-8')
                return {} if not raw else json.loads(raw)

            def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
                encoded = json.dumps(payload, separators=(',', ':')).encode('utf-8')
                self.send_response(int(status))
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_html(self) -> None:
                encoded = HTML_PAGE.encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self) -> None:  # noqa: N802
                try:
                    if self.path == '/' or self.path.startswith('/index.html'):
                        self._send_html()
                    elif self.path.startswith('/api/state'):
                        self._send_json(parent.get_state_snapshot())
                    elif self.path.startswith('/api/map'):
                        self._send_json(parent.get_map_snapshot())
                    elif self.path.startswith('/api/maps'):
                        self._send_json(parent.list_saved_maps())
                    elif self.path.startswith('/api/config'):
                        self._send_json({'ok': True, 'config': parent.get_runtime_config()})
                    elif self.path.startswith('/api/system/status'):
                        self._send_json(parent.call_host_control('status', method='GET'))
                    else:
                        self._send_json({'ok': False, 'error': 'not found'}, HTTPStatus.NOT_FOUND)
                except Exception as exc:  # noqa: BLE001
                    self._send_json({'ok': False, 'error': str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

            def do_POST(self) -> None:  # noqa: N802
                try:
                    payload = self._read_json()
                    if self.path.startswith('/api/mode'):
                        mode = str(payload.get('mode', '')).lower()
                        parent.set_mode(mode)
                        self._send_json({'ok': True, 'mode': mode})
                    elif self.path.startswith('/api/teleop'):
                        parent.set_mode(MODE_TELEOP)
                        parent.set_teleop_from_payload(payload)
                        self._send_json({'ok': True})
                    elif self.path.startswith('/api/stop'):
                        parent.set_mode(MODE_ESTOP)
                        parent.cancel_nav_goal()
                        parent.publish_zero_now()
                        self._send_json({'ok': True, 'mode': MODE_ESTOP})
                    elif self.path.startswith('/api/clear_estop'):
                        parent.set_mode(MODE_TELEOP)
                        parent.publish_zero_now()
                        self._send_json({'ok': True, 'mode': MODE_TELEOP})
                    elif self.path.startswith('/api/goal'):
                        x = float(payload['x'])
                        y = float(payload['y'])
                        yaw = float(payload.get('yaw', 0.0))
                        parent.set_mode(MODE_WAYPOINT)
                        parent.send_nav_goal(x, y, yaw)
                        self._send_json({'ok': True, 'mode': MODE_WAYPOINT, 'goal': {'x': x, 'y': y, 'yaw': yaw}})
                    elif self.path.startswith('/api/cancel'):
                        parent.cancel_nav_goal()
                        parent.publish_zero_now()
                        self._send_json({'ok': True})
                    elif self.path.startswith('/api/config'):
                        parent.update_runtime_config(payload)
                        self._send_json({'ok': True, 'config': parent.get_runtime_config()})
                    elif self.path.startswith('/api/map/save'):
                        self._send_json(parent.save_current_map(str(payload.get('name', '')).strip()))
                    elif self.path.startswith('/api/map/load'):
                        self._send_json(parent.load_saved_map(str(payload.get('name', '')).strip()))
                    elif self.path.startswith('/api/map/delete'):
                        self._send_json(parent.delete_saved_map(str(payload.get('name', '')).strip()))
                    elif self.path.startswith('/api/mapping/enable'):
                        self._send_json(parent.set_mapping_updates_enabled(True))
                    elif self.path.startswith('/api/mapping/disable'):
                        self._send_json(parent.set_mapping_updates_enabled(False))
                    elif self.path.startswith('/api/system/restart_orbslam2'):
                        self._send_json(parent.call_host_control('restart/orbslam2'))
                    elif self.path.startswith('/api/system/restart_microros_agent'):
                        self._send_json(parent.call_host_control('restart/microros_agent'))
                    else:
                        self._send_json({'ok': False, 'error': 'not found'}, HTTPStatus.NOT_FOUND)
                except Exception as exc:  # noqa: BLE001
                    self._send_json({'ok': False, 'error': str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        return ThreadingHTTPServer((self.bind_address, self.http_port), Handler)

    def _create_map_subscriptions(self) -> None:
        reliable_transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_topics = [(self.map_topic, 'map')]
        if self.map_fallback_to_costmaps:
            map_topics.extend([
                (self.global_costmap_topic, 'global_costmap'),
                (self.local_costmap_topic, 'local_costmap'),
            ])
        for topic, source in map_topics:
            for qos in (reliable_transient_qos, reliable_volatile_qos, best_effort_volatile_qos):
                self._map_subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        topic,
                        lambda msg, source=source: self._map_callback(msg, source),
                        qos))

    def _nav_cmd_callback(self, msg: Twist) -> None:
        with self._lock:
            self._last_nav_cmd = msg
            self._last_nav_time = time.monotonic()

    def _map_callback(self, msg: OccupancyGrid, source: str = 'map') -> None:
        with self._lock:
            if self._latest_map_source == 'map' and source != 'map':
                return
            self._latest_map = msg
            self._latest_map_source = source
            self._last_map_time = time.monotonic()

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        with self._lock:
            self._latest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _compact_odom_callback(self, msg: MotorOutputOdometryArray) -> None:
        with self._lock:
            if math.isfinite(float(msg.battery_voltage_v)) and float(msg.battery_voltage_v) > 0.0:
                self._latest_battery_v = float(msg.battery_voltage_v)

    def _motor_state_callback(self, msg: HexapodMotorState) -> None:
        with self._lock:
            if len(msg.phase_rad) >= 6:
                self._latest_motor_phase_rad = [float(v) for v in msg.phase_rad[:6]]
                self._have_motor_phase = True
            if math.isfinite(float(msg.battery_voltage_v)) and float(msg.battery_voltage_v) > 0.0:
                self._latest_battery_v = float(msg.battery_voltage_v)

    def set_mode(self, mode: str) -> None:
        mode = mode.lower()
        if mode not in VALID_MODES:
            raise ValueError(f"invalid mode '{mode}', expected one of {sorted(VALID_MODES)}")
        with self._lock:
            if mode != self._mode:
                self._mode = mode
                self._last_mode_change_time = time.monotonic()
                self.get_logger().warn(f"operator mode -> {mode}")
                if mode in {MODE_ESTOP, MODE_TELEOP}:
                    self.cancel_nav_goal()
                if mode == MODE_ESTOP:
                    self._teleop_cmd = Twist()

    def set_teleop_from_payload(self, payload: Dict[str, Any]) -> None:
        cmd = Twist()
        scale = 1.0
        if bool(payload.get('fast', False)):
            scale *= self.keyboard_fast_scale
        if bool(payload.get('slow', False)):
            scale *= self.keyboard_slow_scale
        if 'linear' in payload or 'angular' in payload:
            cmd.linear.x = clamp(float(payload.get('linear', 0.0)), -self.max_linear_mps, self.max_linear_mps)
            cmd.angular.z = clamp(float(payload.get('angular', 0.0)), -self.max_angular_radps, self.max_angular_radps)
        else:
            key = str(payload.get('key', '')).lower()
            linear_unit, angular_unit = key_to_units(key)
            cmd.linear.x = clamp(
                linear_unit * self.keyboard_linear_mps * scale,
                -self.max_linear_mps,
                self.max_linear_mps,
            )
            cmd.angular.z = clamp(
                angular_unit * self.keyboard_angular_radps * scale,
                -self.max_angular_radps,
                self.max_angular_radps,
            )
        with self._lock:
            self._teleop_cmd = cmd
            self._last_teleop_time = time.monotonic()

    def send_nav_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._navigate_client.wait_for_server(timeout_sec=0.2):
            self._last_goal_status = 'navigate_to_pose server unavailable'
            self.get_logger().error(self._last_goal_status)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self._last_goal_status = 'sending'
        future = self._navigate_client.send_goal_async(goal, feedback_callback=self._nav_feedback_callback)
        future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future: Any) -> None:
        goal_handle = future.result()
        with self._lock:
            self._goal_handle = goal_handle
            if not goal_handle.accepted:
                self._last_goal_status = 'rejected'
                return
            self._last_goal_status = 'accepted'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg: Any) -> None:
        del feedback_msg
        with self._lock:
            if self._last_goal_status not in {'canceling', 'canceled'}:
                self._last_goal_status = 'executing'

    def _nav_result_callback(self, future: Any) -> None:
        result = future.result()
        status = int(result.status)
        with self._lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._last_goal_status = 'succeeded'
            elif status == GoalStatus.STATUS_CANCELED:
                self._last_goal_status = 'canceled'
            elif status == GoalStatus.STATUS_ABORTED:
                self._last_goal_status = 'aborted'
            else:
                self._last_goal_status = f'status_{status}'
            self._goal_handle = None

    def cancel_nav_goal(self) -> None:
        with self._lock:
            goal_handle = self._goal_handle
            if goal_handle is None:
                return
            self._last_goal_status = 'canceling'
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to cancel nav goal: {exc}")

    def _publish_selected_cmd_timer(self) -> None:
        cmd = self._select_command()
        self._cmd_pub.publish(cmd)
        with self._lock:
            self._last_selected_cmd = cmd

    def _select_command(self) -> Twist:
        now = time.monotonic()
        zero = Twist()
        with self._lock:
            mode = self._mode
            if mode == MODE_ESTOP:
                return zero
            if mode == MODE_TELEOP:
                if self.latch_teleop_commands:
                    return copy_twist(self._teleop_cmd)
                if now - self._last_teleop_time <= self.teleop_timeout_sec:
                    return copy_twist(self._teleop_cmd)
                return zero
            if mode == MODE_WAYPOINT:
                if now - self._last_nav_time <= self.nav_cmd_timeout_sec:
                    cmd = copy_twist(self._last_nav_cmd)
                    cmd.linear.x = clamp(cmd.linear.x, -self.waypoint_max_linear_mps, self.waypoint_max_linear_mps)
                    cmd.angular.z = clamp(cmd.angular.z, -self.waypoint_max_angular_radps, self.waypoint_max_angular_radps)
                    return cmd
                return zero
        return zero

    def publish_zero_now(self) -> None:
        zero = Twist()
        self._cmd_pub.publish(zero)
        with self._lock:
            self._last_selected_cmd = zero
            self._teleop_cmd = zero

    def _publish_status_timer(self) -> None:
        with self._lock:
            battery = self._latest_battery_v
        if battery is not None:
            battery_msg = Float32()
            battery_msg.data = float(battery)
            self._battery_pub.publish(battery_msg)
        if self.publish_status_json:
            msg = String()
            msg.data = json.dumps(self.get_state_snapshot(), separators=(',', ':'))
            self._status_pub.publish(msg)

    def get_runtime_config(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'keyboard_linear_mps': self.keyboard_linear_mps,
                'keyboard_angular_radps': self.keyboard_angular_radps,
                'max_linear_mps': self.max_linear_mps,
                'max_angular_radps': self.max_angular_radps,
                'waypoint_max_linear_mps': self.waypoint_max_linear_mps,
                'waypoint_max_angular_radps': self.waypoint_max_angular_radps,
                'latch_teleop_commands': self.latch_teleop_commands,
                'map_storage_dir': str(self.map_storage_dir),
                'loaded_map_topic': self.loaded_map_topic,
                'host_control_base_url': self.host_control_base_url,
                'mapping_updates_enabled': self._mapping_updates_enabled,
                'mapping_enable_topic': self.mapping_enable_topic,
            }

    def update_runtime_config(self, payload: Dict[str, Any]) -> None:
        with self._lock:
            if 'max_linear_mps' in payload:
                self.max_linear_mps = abs(float(payload['max_linear_mps']))
            if 'max_angular_radps' in payload:
                self.max_angular_radps = abs(float(payload['max_angular_radps']))
            if 'keyboard_linear_mps' in payload:
                self.keyboard_linear_mps = clamp(abs(float(payload['keyboard_linear_mps'])), 0.0, self.max_linear_mps)
            if 'keyboard_angular_radps' in payload:
                self.keyboard_angular_radps = clamp(abs(float(payload['keyboard_angular_radps'])), 0.0, self.max_angular_radps)
            if 'waypoint_max_linear_mps' in payload:
                self.waypoint_max_linear_mps = abs(float(payload['waypoint_max_linear_mps']))
            if 'waypoint_max_angular_radps' in payload:
                self.waypoint_max_angular_radps = abs(float(payload['waypoint_max_angular_radps']))
            if 'latch_teleop_commands' in payload:
                self.latch_teleop_commands = bool(payload['latch_teleop_commands'])

    def set_mapping_updates_enabled(self, enabled: bool) -> Dict[str, Any]:
        enabled = bool(enabled)
        self._publish_mapping_enable(enabled)
        with self._lock:
            self._mapping_updates_enabled = enabled
            self._last_system_action_status = (
                'mapping updates enabled' if enabled else 'mapping updates paused')
            state = self._mapping_updates_enabled
        return {
            'ok': True,
            'mapping_updates_enabled': state,
            'topic': self.mapping_enable_topic,
            'note': 'scan gate updated; Nav2 obstacle scan remains live',
        }

    def _publish_mapping_enable(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        # Publish a few times so a just-started scan gate sees the current state
        # even without transient-local QoS.
        for _ in range(3):
            self._mapping_enable_pub.publish(msg)

    def call_mapping_update_service(self, enabled: bool) -> bool:
        # Backward-compatible API used by older GUI code. The current stack uses
        # scan_gate_node rather than slam_toolbox pause/resume services.
        self.set_mapping_updates_enabled(enabled)
        return True

    def call_host_control(self, action: str, method: str = 'POST') -> Dict[str, Any]:
        if not self.host_control_base_url:
            return {'ok': False, 'error': 'host_control_base_url is empty'}
        url = f"{self.host_control_base_url}/{action.lstrip('/')}"
        request = urllib.request.Request(url, method=method)
        request.add_header('Content-Type', 'application/json')
        if method.upper() == 'POST':
            request.data = b'{}'
        try:
            with urllib.request.urlopen(request, timeout=self.host_control_timeout_sec) as response:
                text = response.read().decode('utf-8')
                payload = json.loads(text) if text else {'ok': True}
                with self._lock:
                    self._last_system_action_status = str(payload.get('message') or payload.get('status') or payload)
                return payload
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            error = f'host control HTTP {exc.code}: {body}'
        except Exception as exc:  # noqa: BLE001
            error = f'host control failed: {exc}'
        with self._lock:
            self._last_system_action_status = error
        self.get_logger().error(error)
        return {'ok': False, 'error': error, 'url': url}

    def list_saved_maps(self) -> Dict[str, Any]:
        self.map_storage_dir.mkdir(parents=True, exist_ok=True)
        maps = []
        for yaml_path in sorted(self.map_storage_dir.glob('*.yaml')):
            pgm_path = yaml_path.with_suffix('.pgm')
            maps.append({
                'name': yaml_path.stem,
                'yaml': str(yaml_path),
                'pgm': str(pgm_path),
                'has_pgm': pgm_path.exists(),
                'mtime_sec': yaml_path.stat().st_mtime,
            })
        return {'ok': True, 'directory': str(self.map_storage_dir), 'maps': maps}

    def save_current_map(self, name: str) -> Dict[str, Any]:
        safe_name = sanitize_map_name(name)
        if not safe_name:
            return {'ok': False, 'error': 'map name is empty or invalid'}
        with self._lock:
            msg = self._latest_map
        if msg is None:
            return {'ok': False, 'error': 'no occupancy map has been received'}
        yaml_path = self.map_storage_dir / f'{safe_name}.yaml'
        pgm_path = self.map_storage_dir / f'{safe_name}.pgm'
        try:
            write_map_files(msg, yaml_path, pgm_path, self.map_occupied_thresh, self.map_free_thresh)
            return {'ok': True, 'name': safe_name, 'yaml': str(yaml_path), 'pgm': str(pgm_path), 'directory': str(self.map_storage_dir)}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to save map '{safe_name}': {exc}")
            return {'ok': False, 'error': str(exc)}

    def load_saved_map(self, name: str) -> Dict[str, Any]:
        safe_name = sanitize_map_name(name)
        if not safe_name:
            return {'ok': False, 'error': 'map name is empty or invalid'}
        yaml_path = self.map_storage_dir / f'{safe_name}.yaml'
        try:
            msg = read_map_files(yaml_path, self.get_clock().now().to_msg())
            with self._lock:
                self._latest_map = msg
                self._latest_map_source = 'loaded_map'
                self._last_map_time = time.monotonic()
            self._loaded_map_pub.publish(msg)
            return {'ok': True, 'name': safe_name, 'yaml': str(yaml_path), 'published_topic': self.loaded_map_topic}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to load map '{safe_name}': {exc}")
            return {'ok': False, 'error': str(exc)}

    def delete_saved_map(self, name: str) -> Dict[str, Any]:
        safe_name = sanitize_map_name(name)
        if not safe_name:
            return {'ok': False, 'error': 'map name is empty or invalid'}
        removed = []
        for suffix in ('.yaml', '.pgm'):
            path = self.map_storage_dir / f'{safe_name}{suffix}'
            if path.exists():
                path.unlink()
                removed.append(str(path))
        return {'ok': True, 'name': safe_name, 'removed': removed}

    def get_robot_skeleton_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            phase = list(self._latest_motor_phase_rad)
            have_phase = self._have_motor_phase
        half_l = 0.5 * self.robot_body_length_m
        half_w = 0.5 * self.robot_body_width_m
        mounts = [
            ('RF', half_l * 0.72, -half_w),
            ('RM', 0.0, -half_w),
            ('RB', -half_l * 0.72, -half_w),
            ('LF', half_l * 0.72, half_w),
            ('LM', 0.0, half_w),
            ('LB', -half_l * 0.72, half_w),
        ]
        legs = []
        for index, (label, mount_x, mount_y) in enumerate(mounts):
            side_sign = 1.0 if mount_y > 0.0 else -1.0
            p = phase[index] if index < len(phase) else 0.0
            legs.append({
                'label': label,
                'mount_x': mount_x,
                'mount_y': mount_y,
                'foot_x': mount_x + self.robot_stride_visual_m * math.cos(p),
                'foot_y': mount_y + side_sign * self.robot_leg_lateral_visual_m,
                'phase_rad': p,
            })
        return {
            'frame': 'base_link',
            'units': 'm',
            'have_phase': have_phase,
            'body': {
                'length_m': self.robot_body_length_m,
                'width_m': self.robot_body_width_m,
                'corners': [
                    {'x': half_l, 'y': half_w},
                    {'x': half_l, 'y': -half_w},
                    {'x': -half_l, 'y': -half_w},
                    {'x': -half_l, 'y': half_w},
                ],
            },
            'legs': legs,
        }

    def get_state_snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            pose = None
            if self._latest_pose is not None:
                pose = {'x': self._latest_pose[0], 'y': self._latest_pose[1], 'yaw': self._latest_pose[2]}
            selected = {'linear_x': self._last_selected_cmd.linear.x, 'angular_z': self._last_selected_cmd.angular.z}
            teleop = {
                'linear_x': self._teleop_cmd.linear.x,
                'angular_z': self._teleop_cmd.angular.z,
                'age_sec': now - self._last_teleop_time if self._last_teleop_time > 0.0 else None,
            }
            nav_age = now - self._last_nav_time if self._last_nav_time > 0.0 else None
            has_map = self._latest_map is not None
            map_info = None
            if self._latest_map is not None:
                info = self._latest_map.info
                map_info = {
                    'width': int(info.width),
                    'height': int(info.height),
                    'resolution': float(info.resolution),
                    'origin_x': float(info.origin.position.x),
                    'origin_y': float(info.origin.position.y),
                }
            return {
                'ok': True,
                'mode': self._mode,
                'battery_voltage_v': self._latest_battery_v,
                'pose': pose,
                'selected_cmd': selected,
                'teleop_cmd': teleop,
                'nav_cmd_age_sec': nav_age,
                'nav_goal_status': self._last_goal_status,
                'system_action_status': self._last_system_action_status,
                'mapping_updates_enabled': self._mapping_updates_enabled,
                'mapping_enable_topic': self.mapping_enable_topic,
                'has_map': has_map,
                'map_source': self._latest_map_source if has_map else None,
                'map_age_sec': now - self._last_map_time if self._last_map_time > 0.0 else None,
                'map_info': map_info,
                'teleop_latched': self.latch_teleop_commands,
                'config': self.get_runtime_config(),
                'robot_skeleton': self.get_robot_skeleton_snapshot(),
                'topics': {
                    'selected_cmd_vel': self.selected_cmd_vel_topic,
                    'nav_cmd_vel': self.nav_cmd_vel_topic,
                    'map': self.map_topic,
                    'global_costmap': self.global_costmap_topic,
                    'local_costmap': self.local_costmap_topic,
                    'odom': self.odom_topic,
                    'compact_odom': self.compact_odom_topic,
                    'motor_state': self.motor_state_topic,
                    'loaded_map': self.loaded_map_topic,
                },
            }

    def get_map_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            msg = self._latest_map
            pose = self._latest_pose
        if msg is None:
            return {'ok': False, 'error': 'no map received yet'}
        info = msg.info
        return {
            'ok': True,
            'source': self._latest_map_source,
            'age_sec': time.monotonic() - self._last_map_time if self._last_map_time > 0.0 else None,
            'header': {
                'frame_id': msg.header.frame_id,
                'stamp_sec': int(msg.header.stamp.sec),
                'stamp_nanosec': int(msg.header.stamp.nanosec),
            },
            'info': {
                'width': int(info.width),
                'height': int(info.height),
                'resolution': float(info.resolution),
                'origin': {
                    'x': float(info.origin.position.x),
                    'y': float(info.origin.position.y),
                    'yaw': quaternion_to_yaw(
                        info.origin.orientation.x,
                        info.origin.orientation.y,
                        info.origin.orientation.z,
                        info.origin.orientation.w,
                    ),
                },
            },
            'pose': None if pose is None else {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            'robot_skeleton': self.get_robot_skeleton_snapshot(),
            'data': list(msg.data),
        }


def sanitize_map_name(name: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', name.strip()).strip('._-')[:80]


def write_map_files(msg: OccupancyGrid, yaml_path: Path, pgm_path: Path, occupied_thresh: float, free_thresh: float) -> None:
    del free_thresh
    info = msg.info
    width = int(info.width)
    height = int(info.height)
    if width <= 0 or height <= 0:
        raise ValueError('map has invalid dimensions')
    data = list(msg.data)
    if len(data) != width * height:
        raise ValueError(f'map data length {len(data)} does not match {width}x{height}')
    occupied_value = int(round(occupied_thresh * 100.0))
    image = bytearray()
    for image_y in range(height):
        map_y = height - 1 - image_y
        for x in range(width):
            value = int(data[(map_y * width) + x])
            if value < 0:
                image.append(205)
            elif value >= occupied_value:
                image.append(0)
            else:
                image.append(254)
    with pgm_path.open('wb') as handle:
        handle.write(f'P5\n# saved by hexapod_operator_interface\n{width} {height}\n255\n'.encode('ascii'))
        handle.write(image)
    yaw = quaternion_to_yaw(
        info.origin.orientation.x,
        info.origin.orientation.y,
        info.origin.orientation.z,
        info.origin.orientation.w,
    )
    yaml_path.write_text(
        f'image: {pgm_path.name}\n'
        f'mode: trinary\n'
        f'resolution: {float(info.resolution):.9g}\n'
        f'origin: [{float(info.origin.position.x):.9g}, {float(info.origin.position.y):.9g}, {yaw:.9g}]\n'
        f'negate: 0\n'
        f'occupied_thresh: {occupied_thresh:.9g}\n'
        f'free_thresh: 0.25\n')


def read_map_files(yaml_path: Path, stamp: Any) -> OccupancyGrid:
    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))
    parsed = parse_map_yaml(yaml_path.read_text())
    image_path = yaml_path.parent / parsed['image']
    width, height, pixels = read_pgm_p5(image_path)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = float(parsed.get('resolution', 0.05))
    origin = parsed.get('origin', [0.0, 0.0, 0.0])
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    qx, qy, qz, qw = yaw_to_quaternion(float(origin[2]))
    msg.info.origin.orientation.x = qx
    msg.info.origin.orientation.y = qy
    msg.info.origin.orientation.z = qz
    msg.info.origin.orientation.w = qw
    data = [0] * (width * height)
    for image_y in range(height):
        map_y = height - 1 - image_y
        for x in range(width):
            pix = pixels[(image_y * width) + x]
            if 190 <= pix <= 220:
                value = -1
            elif pix < 128:
                value = 100
            else:
                value = 0
            data[(map_y * width) + x] = value
    msg.data = data
    return msg


def parse_map_yaml(text: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for raw_line in text.splitlines():
        line = raw_line.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip().strip('"\'')
        if key == 'origin':
            nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', value)
            result[key] = [float(v) for v in nums[:3]]
        elif key in {'resolution', 'occupied_thresh', 'free_thresh'}:
            result[key] = float(value)
        elif key == 'negate':
            result[key] = int(value)
        else:
            result[key] = value
    if 'image' not in result:
        raise ValueError('map YAML does not contain image field')
    if 'origin' not in result:
        result['origin'] = [0.0, 0.0, 0.0]
    return result


def read_pgm_p5(path: Path) -> Tuple[int, int, bytes]:
    raw = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(raw) and chr(raw[index]).isspace():
            index += 1
        if index < len(raw) and raw[index] == ord('#'):
            while index < len(raw) and raw[index] not in (10, 13):
                index += 1
            continue
        start = index
        while index < len(raw) and not chr(raw[index]).isspace():
            index += 1
        tokens.append(raw[start:index].decode('ascii'))
    if tokens[0] != 'P5':
        raise ValueError('only binary P5 PGM maps are supported')
    width = int(tokens[1])
    height = int(tokens[2])
    maxval = int(tokens[3])
    if maxval != 255:
        raise ValueError('only 8-bit PGM maps are supported')
    while index < len(raw) and chr(raw[index]).isspace():
        index += 1
    pixels = raw[index:index + (width * height)]
    if len(pixels) != width * height:
        raise ValueError('PGM data is truncated')
    return width, height, pixels


def key_to_units(key: str) -> Tuple[float, float]:
    mapping = {
        'q': (1.0, 1.0),
        'w': (1.0, 0.0),
        'e': (1.0, -1.0),
        'a': (0.0, 1.0),
        's': (0.0, 0.0),
        'd': (0.0, -1.0),
        'z': (-1.0, -1.0),
        'x': (-1.0, 0.0),
        'c': (-1.0, 1.0),
        ' ': (0.0, 0.0),
    }
    return mapping.get(key, (0.0, 0.0))


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def copy_twist(msg: Twist) -> Twist:
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


HTML_PAGE = r'''<!doctype html>
<html><head><meta charset="utf-8"/><title>Hexapod Operator</title><meta name="viewport" content="width=device-width, initial-scale=1"/>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}header{padding:12px 16px;background:#1b1b1b;display:flex;gap:16px;align-items:center;flex-wrap:wrap}button{font-size:16px;padding:8px 12px;border-radius:8px;border:1px solid #555;background:#2b2b2b;color:#eee;cursor:pointer}button:hover{background:#3a3a3a}button.stop{background:#8b0000;border-color:#d44;font-weight:700}button.mode.active{outline:3px solid #88c0ff}main{display:grid;grid-template-columns:minmax(420px,1fr) 430px;gap:12px;padding:12px}#mapWrap{background:#000;border:1px solid #444;border-radius:8px;overflow:hidden;position:relative;min-height:520px;touch-action:none}#map{width:100%;height:78vh;display:block;background:#222;cursor:grab}#map.dragging{cursor:grabbing}.map-controls{position:absolute;left:12px;top:12px;display:flex;gap:6px;flex-wrap:wrap;z-index:2;background:rgba(0,0,0,.62);border:1px solid #444;border-radius:8px;padding:6px}.map-controls button{font-size:13px;padding:5px 8px}.map-help{position:absolute;left:12px;bottom:12px;z-index:2;background:rgba(0,0,0,.62);border:1px solid #444;border-radius:8px;padding:6px 8px;color:#ddd;font-size:12px}.panel{background:#1b1b1b;border:1px solid #444;border-radius:8px;padding:12px;overflow-y:auto;max-height:calc(100vh - 94px)}.row{display:flex;gap:8px;margin:8px 0;flex-wrap:wrap;align-items:center}.big{font-size:28px;font-weight:700}.grid3{display:grid;grid-template-columns:repeat(3,72px);gap:8px;justify-content:center}.grid3 button{height:54px;font-size:22px}code{color:#aad}.small{color:#bbb;font-size:13px;line-height:1.35}input,select{width:94px;padding:6px;border-radius:6px;border:1px solid #555;background:#111;color:#eee}select{width:180px}.wide-input{width:170px}.status-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.status-card{background:#111;border:1px solid #333;border-radius:8px;padding:10px;min-height:58px}.status-card.wide{grid-column:1/-1}.status-label{color:#999;font-size:12px;text-transform:uppercase;letter-spacing:.05em}.status-value{font-size:18px;margin-top:4px;overflow-wrap:anywhere}.status-ok{color:#86efac}.status-warn{color:#facc15}.status-bad{color:#fca5a5}#birdseye{width:100%;height:260px;background:#070707;border:1px solid #333;border-radius:8px}.section-title{margin-top:18px;border-top:1px solid #333;padding-top:12px}label{color:#ccc;font-size:13px}
</style></head><body>
<header><span class="big">Hexapod Operator</span><button class="mode" id="modeTeleop" onclick="setMode('teleop')">Teleop</button><button class="mode" id="modeWaypoint" onclick="setMode('waypoint')">Waypoint</button><button class="stop" onclick="stopNow()">STOP / ESTOP</button><button onclick="clearEstop()">Clear to Teleop</button><span>Mode: <code id="mode">?</code></span><span>Battery: <code id="battery">?</code> V</span><span>Nav: <code id="navStatus">?</code></span></header>
<main><section id="mapWrap"><div class="map-controls"><button onclick="fitMapView()">Fit map</button><button onclick="centerRobotView()">Center robot</button><button onclick="zoomMapButton(1.25)">+</button><button onclick="zoomMapButton(0.8)">−</button></div><canvas id="map"></canvas><div class="map-help">Wheel: zoom · Drag: pan · Click: waypoint</div></section><aside class="panel">
<h2>Birdseye Hexapod</h2><canvas id="birdseye"></canvas><p class="small">Top-down body outline and six leg/foot positions are drawn to scale in meters from the latest motor-state phases.</p>
<h2 class="section-title">Teleop</h2><p class="small">Keyboard: q/w/e, a/s/d, z/x/c. Commands latch by default; press S, Space, or STOP to stop. Shift/Alt apply fast/slow scaling.</p>
<div class="grid3"><button onmousedown="pressKey('q',event)" onmouseup="releaseKey('q')">q</button><button onmousedown="pressKey('w',event)" onmouseup="releaseKey('w')">w</button><button onmousedown="pressKey('e',event)" onmouseup="releaseKey('e')">e</button><button onmousedown="pressKey('a',event)" onmouseup="releaseKey('a')">a</button><button onmousedown="pressKey('s',event)">s</button><button onmousedown="pressKey('d',event)" onmouseup="releaseKey('d')">d</button><button onmousedown="pressKey('z',event)" onmouseup="releaseKey('z')">z</button><button onmousedown="pressKey('x',event)" onmouseup="releaseKey('x')">x</button><button onmousedown="pressKey('c',event)" onmouseup="releaseKey('c')">c</button></div>
<div class="row"><label>teleop vx <input id="teleopLinear" type="number" step="0.005"></label><label>teleop wz <input id="teleopAngular" type="number" step="0.01"></label></div><div class="row"><label>teleop max vx <input id="teleopMaxLinear" type="number" step="0.005"></label><label>teleop max wz <input id="teleopMaxAngular" type="number" step="0.01"></label></div><div class="row"><button onclick="saveVelocityConfig()">Apply Speeds</button><button onclick="sendManualTeleop()">Send Manual Teleop</button></div>
<h2 class="section-title">Waypoint</h2><p class="small">Click the map to place a Nav2 goal. Max waypoint velocities are enforced in the operator arbiter before commands reach the tripod controller.</p><div class="row">x <input id="goalX" type="number" step="0.01"> y <input id="goalY" type="number" step="0.01"> yaw <input id="goalYaw" type="number" step="0.01" value="0.0"></div><div class="row"><label>wp max vx <input id="waypointMaxLinear" type="number" step="0.005"></label><label>wp max wz <input id="waypointMaxAngular" type="number" step="0.01"></label></div><div class="row"><button onclick="sendGoalFields()">Send Goal</button><button onclick="cancelNav()">Cancel Nav</button></div>
<h2 class="section-title">Mapping / System</h2><p class="small">Pause mapping to freeze occupancy-grid growth while keeping the displayed map available. Restart buttons call a host-only helper service on the Jetson.</p><div class="row"><button onclick="enableMapping()">Enable map updates</button><button onclick="disableMapping()">Pause map updates</button></div><div class="row"><button onclick="restartOrbslam2()">Restart ORB-SLAM2</button><button onclick="restartMicrorosAgent()">Restart micro-ROS Agent</button></div><div id="systemActionStatus" class="small"></div>
<h2 class="section-title">Map Storage</h2><p class="small">Maps are saved as YAML+PGM under <code id="mapDir">?</code>. This defaults to /ros2_ws/maps for a Jetson/container bind mount.</p><div class="row"><input id="mapName" class="wide-input" placeholder="stage_map_01"><button onclick="saveMap()">Save current map</button></div><div class="row"><select id="mapSelect"></select><button onclick="loadMap()">Load</button><button onclick="deleteMap()">Delete</button><button onclick="refreshMaps()">Refresh</button></div><div id="mapStorageStatus" class="small"></div>
<h2 class="section-title">Status</h2><div id="statusCards" class="status-grid"></div></aside></main>
<script>
let lastMap=null,latestState=null,heldKey=null,heldFast=false,heldSlow=false,teleopLatched=true,configInitialized=false;
let mapView={scale:null,offsetX:0,offsetY:0,image:null,imageKey:'',dragging:false,dragMoved:false,dragStartX:0,dragStartY:0,dragOffsetX:0,dragOffsetY:0,fitOnNextMap:true};
async function post(path,payload={}){const res=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});return await res.json();}
async function setMode(mode){await post('/api/mode',{mode});await refreshState()} async function stopNow(){await post('/api/stop',{});heldKey=null;await refreshState()} async function clearEstop(){await post('/api/clear_estop',{});await refreshState()} async function cancelNav(){await post('/api/cancel',{});await refreshState()} async function sendKey(key,ev=null){await post('/api/teleop',{key,fast:ev?ev.shiftKey:false,slow:ev?ev.altKey:false})}
async function enableMapping(){const r=await post('/api/mapping/enable',{});document.getElementById('systemActionStatus').textContent=r.ok?'Mapping updates enabled/requested':`Mapping enable failed: ${r.error||r.note}`;await refreshState()}
async function disableMapping(){const r=await post('/api/mapping/disable',{});document.getElementById('systemActionStatus').textContent=r.ok?'Mapping updates paused/requested':`Mapping pause failed: ${r.error||r.note}`;await refreshState()}
async function restartOrbslam2(){if(!confirm('Restart bare-metal ORB-SLAM2 service?'))return;const r=await post('/api/system/restart_orbslam2',{});document.getElementById('systemActionStatus').textContent=r.ok?'ORB-SLAM2 restart requested':`ORB-SLAM2 restart failed: ${r.error||r.message}`;await refreshState()}
async function restartMicrorosAgent(){if(!confirm('Restart micro-ROS Agent service? Motor communications may briefly drop.'))return;const r=await post('/api/system/restart_microros_agent',{});document.getElementById('systemActionStatus').textContent=r.ok?'micro-ROS Agent restart requested':`micro-ROS restart failed: ${r.error||r.message}`;await refreshState()}
function pressKey(key,ev=null){heldKey=key;heldFast=ev?ev.shiftKey:false;heldSlow=ev?ev.altKey:false;sendKey(key,ev)} function releaseKey(key){if(heldKey===key)heldKey=null;if(!teleopLatched)sendKey('s')}
function sendManualTeleop(){const linear=parseFloat(document.getElementById('teleopLinear').value||'0');const angular=parseFloat(document.getElementById('teleopAngular').value||'0');post('/api/teleop',{linear,angular}).then(refreshState)}
function saveVelocityConfig(){const payload={keyboard_linear_mps:parseFloat(document.getElementById('teleopLinear').value||'0'),keyboard_angular_radps:parseFloat(document.getElementById('teleopAngular').value||'0'),max_linear_mps:parseFloat(document.getElementById('teleopMaxLinear').value||'0'),max_angular_radps:parseFloat(document.getElementById('teleopMaxAngular').value||'0'),waypoint_max_linear_mps:parseFloat(document.getElementById('waypointMaxLinear').value||'0'),waypoint_max_angular_radps:parseFloat(document.getElementById('waypointMaxAngular').value||'0')};post('/api/config',payload).then(refreshState)}
function sendGoalFields(){const x=parseFloat(document.getElementById('goalX').value);const y=parseFloat(document.getElementById('goalY').value);const yaw=parseFloat(document.getElementById('goalYaw').value||'0');post('/api/goal',{x,y,yaw}).then(refreshState)}
async function saveMap(){const name=document.getElementById('mapName').value||`map_${new Date().toISOString().replace(/[:.]/g,'-')}`;const r=await post('/api/map/save',{name});document.getElementById('mapStorageStatus').textContent=r.ok?`Saved ${r.name} to ${r.directory}`:`Save failed: ${r.error}`;await refreshMaps()}
async function loadMap(){const name=document.getElementById('mapSelect').value;const r=await post('/api/map/load',{name});document.getElementById('mapStorageStatus').textContent=r.ok?`Loaded ${r.name}; published ${r.published_topic}`:`Load failed: ${r.error}`;await refreshState();await refreshMap()}
async function deleteMap(){const name=document.getElementById('mapSelect').value;if(!name||!confirm(`Delete saved map ${name}?`))return;const r=await post('/api/map/delete',{name});document.getElementById('mapStorageStatus').textContent=r.ok?`Deleted ${name}`:`Delete failed: ${r.error}`;await refreshMaps()}
async function refreshMaps(){const p=await(await fetch('/api/maps')).json();const sel=document.getElementById('mapSelect');sel.innerHTML='';if(p.ok){document.getElementById('mapDir').textContent=p.directory;for(const m of p.maps){const opt=document.createElement('option');opt.value=m.name;opt.textContent=`${m.name}${m.has_pgm?'':' (missing pgm)'}`;sel.appendChild(opt)}}}
document.addEventListener('keydown',(ev)=>{const tag=(ev.target&&ev.target.tagName)?ev.target.tagName.toLowerCase():'';if(tag==='input'||tag==='select')return;const k=ev.key.toLowerCase();if('qweasdzxc '.includes(k)){ev.preventDefault();heldKey=(k===' ')?'s':k;heldFast=ev.shiftKey;heldSlow=ev.altKey;sendKey(heldKey,ev)}});document.addEventListener('keyup',(ev)=>{const k=ev.key.toLowerCase();if(heldKey&&(k===heldKey||k===' ')){heldKey=null;if(!teleopLatched)sendKey('s')}});setInterval(()=>{if(heldKey&&!teleopLatched)post('/api/teleop',{key:heldKey,fast:heldFast,slow:heldSlow})},100);
async function refreshState(){const s=await(await fetch('/api/state')).json();latestState=s;document.getElementById('mode').textContent=s.mode;document.getElementById('battery').textContent=s.battery_voltage_v==null?'?':s.battery_voltage_v.toFixed(2);teleopLatched=!!s.teleop_latched;document.getElementById('navStatus').textContent=s.nav_goal_status;document.getElementById('systemActionStatus').textContent=s.system_action_status||'';if(s.config&&!configInitialized){document.getElementById('teleopLinear').value=s.config.keyboard_linear_mps;document.getElementById('teleopAngular').value=s.config.keyboard_angular_radps;document.getElementById('teleopMaxLinear').value=s.config.max_linear_mps;document.getElementById('teleopMaxAngular').value=s.config.max_angular_radps;document.getElementById('waypointMaxLinear').value=s.config.waypoint_max_linear_mps;document.getElementById('waypointMaxAngular').value=s.config.waypoint_max_angular_radps;document.getElementById('mapDir').textContent=s.config.map_storage_dir;configInitialized=true}renderStatusCards(s);drawBirdseye(s.robot_skeleton||(lastMap&&lastMap.robot_skeleton));document.getElementById('modeTeleop').classList.toggle('active',s.mode==='teleop');document.getElementById('modeWaypoint').classList.toggle('active',s.mode==='waypoint')}
function fmt(v,d=2){if(v===null||v===undefined)return '—';if(typeof v==='number')return Number.isFinite(v)?v.toFixed(d):'—';return String(v)} function statusClass(ok,warn=false){if(!ok)return'status-bad';return warn?'status-warn':'status-ok'} function addStatusCard(html,label,value,cls='',wide=false){html.push(`<div class="status-card ${wide?'wide':''}"><div class="status-label">${label}</div><div class="status-value ${cls}">${value}</div></div>`)}
function renderStatusCards(s){const h=[];addStatusCard(h,'Mode',s.mode,s.mode==='estop'?'status-bad':'status-ok');addStatusCard(h,'Battery',`${fmt(s.battery_voltage_v)} V`,s.battery_voltage_v==null?'status-warn':'status-ok');addStatusCard(h,'Map',s.has_map?`${s.map_source} (${fmt(s.map_age_sec,1)} s)`:'not received',statusClass(s.has_map));addStatusCard(h,'Nav goal',s.nav_goal_status||'idle',s.nav_goal_status==='aborted'?'status-bad':'');addStatusCard(h,'Mapping',s.mapping_updates_enabled?'updating':'paused',s.mapping_updates_enabled?'status-ok':'status-warn');addStatusCard(h,'System action',s.system_action_status||'idle','',true);const pose=s.pose?`x ${fmt(s.pose.x,3)} / y ${fmt(s.pose.y,3)} / yaw ${fmt(s.pose.yaw,2)}`:'not received';addStatusCard(h,'Pose',pose,s.pose?'status-ok':'status-warn',true);const cmd=s.selected_cmd?`vx ${fmt(s.selected_cmd.linear_x,3)} m/s / wz ${fmt(s.selected_cmd.angular_z,3)} rad/s`:'—';addStatusCard(h,'Selected command',cmd,'',true);const mapInfo=s.map_info?`${s.map_info.width}×${s.map_info.height}, ${fmt(s.map_info.resolution,3)} m/cell, origin ${fmt(s.map_info.origin_x,2)}, ${fmt(s.map_info.origin_y,2)}`:'—';addStatusCard(h,'Map metadata',mapInfo,'',true);addStatusCard(h,'Teleop latch',s.teleop_latched?'enabled':'disabled',s.teleop_latched?'status-ok':'status-warn');addStatusCard(h,'Waypoint max',`${fmt(s.config?.waypoint_max_linear_mps,3)} m/s / ${fmt(s.config?.waypoint_max_angular_radps,3)} rad/s`,'');document.getElementById('statusCards').innerHTML=h.join('')}
async function refreshMap(){try{const m=await(await fetch('/api/map')).json();if(m.ok){lastMap=m;drawMap()}else{drawNoMap(m.error||'no map received yet')}}catch(err){drawNoMap(String(err))}}
function resizeMapCanvas(){const c=document.getElementById('map'),wrap=document.getElementById('mapWrap');const w=wrap.clientWidth,h=Math.max(440,Math.floor(window.innerHeight*.78));if(c.width!==w)c.width=w;if(c.height!==h)c.height=h;return c}
function drawNoMap(message){const c=resizeMapCanvas(),ctx=c.getContext('2d');ctx.fillStyle='#222';ctx.fillRect(0,0,c.width,c.height);ctx.fillStyle='#ddd';ctx.font='18px system-ui,sans-serif';ctx.fillText('No occupancy map displayed yet',24,42);ctx.font='14px system-ui,sans-serif';ctx.fillText(message,24,70);ctx.fillText('Wheel zoom and drag pan will be active once a map is received.',24,96)}
function makeMapImage(){if(!lastMap)return null;const key=`${lastMap.info.width}x${lastMap.info.height}:${lastMap.map_source||''}:${lastMap.map_age_sec||0}:${lastMap.data.length}`;if(mapView.image&&mapView.imageKey===key)return mapView.image;const w=lastMap.info.width,h=lastMap.info.height,imgCanvas=document.createElement('canvas');imgCanvas.width=w;imgCanvas.height=h;const img=imgCanvas.getContext('2d').createImageData(w,h);for(let y=0;y<h;y++){for(let x=0;x<w;x++){const mi=y*w+x,ii=((h-1-y)*w+x)*4,v=lastMap.data[mi];let col=128;if(v<0)col=70;else if(v===0)col=235;else col=Math.max(0,220-Math.floor(v*2.2));img.data[ii]=col;img.data[ii+1]=col;img.data[ii+2]=col;img.data[ii+3]=255}}imgCanvas.getContext('2d').putImageData(img,0,0);mapView.image=imgCanvas;mapView.imageKey=key;return imgCanvas}
function clampMapScale(scale){if(!lastMap)return scale;const fit=Math.min(document.getElementById('map').width/lastMap.info.width,document.getElementById('map').height/lastMap.info.height);return Math.max(fit*0.35,Math.min(fit*80.0,scale))}
function fitMapView(){if(!lastMap)return;const c=resizeMapCanvas(),w=lastMap.info.width,h=lastMap.info.height;mapView.scale=clampMapScale(Math.min(c.width/w,c.height/h));mapView.offsetX=(c.width-(w*mapView.scale))/2;mapView.offsetY=(c.height-(h*mapView.scale))/2;mapView.fitOnNextMap=false;drawMap()}
function centerRobotView(){if(!lastMap||!lastMap.pose)return;const c=resizeMapCanvas();if(mapView.scale==null)fitMapView();const res=lastMap.info.resolution,mx=(lastMap.pose.x-lastMap.info.origin.x)/res,my=lastMap.info.height-((lastMap.pose.y-lastMap.info.origin.y)/res);mapView.offsetX=(c.width/2)-(mx*mapView.scale);mapView.offsetY=(c.height/2)-(my*mapView.scale);mapView.fitOnNextMap=false;drawMap()}
function zoomMapAt(canvasX,canvasY,factor){if(!lastMap)return;const c=resizeMapCanvas();if(mapView.scale==null)fitMapView();const oldScale=mapView.scale,mapX=(canvasX-mapView.offsetX)/oldScale,mapY=(canvasY-mapView.offsetY)/oldScale;const newScale=clampMapScale(oldScale*factor);mapView.scale=newScale;mapView.offsetX=canvasX-(mapX*newScale);mapView.offsetY=canvasY-(mapY*newScale);mapView.fitOnNextMap=false;drawMap()}
function zoomMapButton(factor){const c=resizeMapCanvas();zoomMapAt(c.width/2,c.height/2,factor)}
function drawMap(){if(!lastMap)return;const c=resizeMapCanvas(),ctx=c.getContext('2d'),w=lastMap.info.width,h=lastMap.info.height,img=makeMapImage();if(mapView.scale==null||mapView.fitOnNextMap)fitMapView();ctx.clearRect(0,0,c.width,c.height);ctx.fillStyle='#080808';ctx.fillRect(0,0,c.width,c.height);ctx.imageSmoothingEnabled=false;ctx.drawImage(img,mapView.offsetX,mapView.offsetY,w*mapView.scale,h*mapView.scale);c._mapDraw={ox:mapView.offsetX,oy:mapView.offsetY,scale:mapView.scale,drawW:w*mapView.scale,drawH:h*mapView.scale};drawRobotOnMap(ctx,mapView.offsetX,mapView.offsetY,mapView.scale);ctx.fillStyle='rgba(0,0,0,.65)';ctx.fillRect(c.width-158,10,148,26);ctx.fillStyle='#ddd';ctx.font='12px system-ui,sans-serif';ctx.fillText(`zoom ${(mapView.scale/Math.min(c.width/w,c.height/h)).toFixed(2)}×`,c.width-146,28)}
function worldToCanvas(wx,wy,ox,oy,scale){const res=lastMap.info.resolution,mx=(wx-lastMap.info.origin.x)/res,my=(wy-lastMap.info.origin.y)/res;return{x:ox+mx*scale,y:oy+(lastMap.info.height-my)*scale}} function robotToWorld(local,pose){const c=Math.cos(pose.yaw),s=Math.sin(pose.yaw);return{x:pose.x+c*local.x-s*local.y,y:pose.y+s*local.x+c*local.y}} function worldToCanvasPoint(p,ox,oy,scale){return worldToCanvas(p.x,p.y,ox,oy,scale)}
function drawRobotOnMap(ctx,ox,oy,scale){if(!lastMap||!lastMap.pose||!lastMap.robot_skeleton)return;const pose=lastMap.pose,sk=lastMap.robot_skeleton,corners=sk.body.corners.map(c=>worldToCanvasPoint(robotToWorld(c,pose),ox,oy,scale));const lw=Math.max(2,Math.min(5,scale*0.08));ctx.strokeStyle='#00aaff';ctx.lineWidth=lw;ctx.fillStyle='rgba(0,170,255,.18)';ctx.beginPath();corners.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fill();ctx.stroke();for(const leg of sk.legs||[]){const m=worldToCanvasPoint(robotToWorld({x:leg.mount_x,y:leg.mount_y},pose),ox,oy,scale),f=worldToCanvasPoint(robotToWorld({x:leg.foot_x,y:leg.foot_y},pose),ox,oy,scale);ctx.strokeStyle='#ffd166';ctx.lineWidth=Math.max(1.5,Math.min(4,scale*0.05));ctx.beginPath();ctx.moveTo(m.x,m.y);ctx.lineTo(f.x,f.y);ctx.stroke();ctx.fillStyle='#ff6b35';ctx.beginPath();ctx.arc(f.x,f.y,Math.max(3,Math.min(8,scale*0.09)),0,2*Math.PI);ctx.fill()}const nose=worldToCanvasPoint(robotToWorld({x:sk.body.length_m*.65,y:0},pose),ox,oy,scale),center=worldToCanvas(pose.x,pose.y,ox,oy,scale);ctx.strokeStyle='#00aaff';ctx.lineWidth=lw;ctx.beginPath();ctx.moveTo(center.x,center.y);ctx.lineTo(nose.x,nose.y);ctx.stroke()}
function canvasToWorld(cx,cy){const d=document.getElementById('map')._mapDraw;if(!lastMap||!d)return null;const mx=(cx-d.ox)/d.scale,my=lastMap.info.height-((cy-d.oy)/d.scale);return{x:lastMap.info.origin.x+mx*lastMap.info.resolution,y:lastMap.info.origin.y+my*lastMap.info.resolution}}
function drawBirdseye(sk){const c=document.getElementById('birdseye'),ctx=c.getContext('2d'),cssW=c.clientWidth||380,cssH=260;c.width=cssW;c.height=cssH;ctx.fillStyle='#070707';ctx.fillRect(0,0,cssW,cssH);if(!sk){ctx.fillStyle='#ddd';ctx.fillText('No skeleton state yet',14,28);return}const maxDim=Math.max(sk.body.width_m+.6,sk.body.length_m+.6),scale=Math.min(cssW,cssH)*.78/maxDim,cx=cssW/2,cy=cssH/2;function pt(x,y){return{x:cx+y*scale,y:cy-x*scale}}const corners=sk.body.corners.map(c=>pt(c.x,c.y));ctx.strokeStyle='#00aaff';ctx.fillStyle='rgba(0,170,255,.16)';ctx.lineWidth=3;ctx.beginPath();corners.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fill();ctx.stroke();ctx.strokeStyle='#ffd166';ctx.lineWidth=2;ctx.font='12px system-ui,sans-serif';for(const leg of sk.legs||[]){const m=pt(leg.mount_x,leg.mount_y),f=pt(leg.foot_x,leg.foot_y);ctx.beginPath();ctx.moveTo(m.x,m.y);ctx.lineTo(f.x,f.y);ctx.stroke();ctx.fillStyle='#ff6b35';ctx.beginPath();ctx.arc(f.x,f.y,5,0,2*Math.PI);ctx.fill();ctx.fillStyle='#ddd';ctx.fillText(leg.label,f.x+5,f.y-5)}const n=pt(sk.body.length_m*.65,0),center=pt(0,0);ctx.strokeStyle='#00aaff';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(center.x,center.y);ctx.lineTo(n.x,n.y);ctx.stroke();ctx.fillStyle=sk.have_phase?'#86efac':'#facc15';ctx.fillText(sk.have_phase?'motor phase live':'phase not received; nominal pose',12,cssH-12)}
const mapCanvas=document.getElementById('map');
mapCanvas.addEventListener('wheel',(ev)=>{if(!lastMap)return;ev.preventDefault();const rect=mapCanvas.getBoundingClientRect(),factor=Math.exp(-ev.deltaY*0.0012);zoomMapAt(ev.clientX-rect.left,ev.clientY-rect.top,factor)},{passive:false});
mapCanvas.addEventListener('pointerdown',(ev)=>{mapView.dragging=true;mapView.dragMoved=false;mapView.dragStartX=ev.clientX;mapView.dragStartY=ev.clientY;mapView.dragOffsetX=mapView.offsetX;mapView.dragOffsetY=mapView.offsetY;mapCanvas.setPointerCapture(ev.pointerId);mapCanvas.classList.add('dragging')});
mapCanvas.addEventListener('pointermove',(ev)=>{if(!mapView.dragging)return;const dx=ev.clientX-mapView.dragStartX,dy=ev.clientY-mapView.dragStartY;if(Math.abs(dx)>2||Math.abs(dy)>2)mapView.dragMoved=true;mapView.offsetX=mapView.dragOffsetX+dx;mapView.offsetY=mapView.dragOffsetY+dy;mapView.fitOnNextMap=false;drawMap()});
function endMapDrag(ev){if(!mapView.dragging)return;mapView.dragging=false;mapCanvas.classList.remove('dragging');try{mapCanvas.releasePointerCapture(ev.pointerId)}catch(e){}}
mapCanvas.addEventListener('pointerup',endMapDrag);mapCanvas.addEventListener('pointercancel',endMapDrag);
mapCanvas.addEventListener('click',(ev)=>{if(!lastMap||mapView.dragMoved)return;const rect=ev.target.getBoundingClientRect(),p=canvasToWorld(ev.clientX-rect.left,ev.clientY-rect.top);if(!p)return;document.getElementById('goalX').value=p.x.toFixed(3);document.getElementById('goalY').value=p.y.toFixed(3);if(confirm(`Send waypoint goal to x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)}?`)){post('/api/goal',{x:p.x,y:p.y,yaw:parseFloat(document.getElementById('goalYaw').value||'0')}).then(refreshState)}});
window.addEventListener('resize',()=>{mapView.fitOnNextMap=true;drawMap();if(latestState)drawBirdseye(latestState.robot_skeleton)});setInterval(refreshState,250);setInterval(refreshMap,1000);refreshState();refreshMap();refreshMaps();
</script></body></html>
'''


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HexapodOperatorInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
