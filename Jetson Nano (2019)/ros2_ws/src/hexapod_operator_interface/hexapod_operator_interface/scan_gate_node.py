#!/usr/bin/env python3
"""Gate ORB-SLAM virtual scans into SLAM Toolbox while keeping Nav2 obstacle scans live."""

from __future__ import annotations

import copy
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class ScanGateNode(Node):
    """Republish /scan_raw to /scan_nav always and /scan only when mapping is enabled."""

    def __init__(self) -> None:
        super().__init__('hexapod_scan_gate')
        self.scan_in_topic = self.declare_parameter('scan_in_topic', '/scan_raw').value
        self.mapping_scan_topic = self.declare_parameter('mapping_scan_topic', '/scan').value
        self.nav_scan_topic = self.declare_parameter('nav_scan_topic', '/scan_nav').value
        self.mapping_enable_topic = self.declare_parameter(
            'mapping_enable_topic', '/hexapod/operator/map_updates_enabled').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/hexapod/operator/scan_gate_status').value
        self.mapping_enabled = bool(self.declare_parameter('mapping_enabled_default', True).value)
        self.nav_publish_enabled = bool(self.declare_parameter('nav_publish_enabled', True).value)
        # Restamp by default. The ORB-SLAM virtual scan can carry a pose/packet timestamp
        # that is older than the current odom TF buffer. SLAM Toolbox then queues and
        # eventually drops every scan, so no /map is produced. Restamping at the gate
        # preserves the geometry while making the scan/TF pair transformable.
        self.restamp_scans = bool(self.declare_parameter('restamp_scans', True).value)
        self.mapping_max_rate_hz = float(self.declare_parameter('mapping_max_rate_hz', 8.0).value)
        self.nav_max_rate_hz = float(self.declare_parameter('nav_max_rate_hz', 12.0).value)
        self.status_rate_hz = float(self.declare_parameter('status_rate_hz', 1.0).value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.mapping_pub = self.create_publisher(LaserScan, self.mapping_scan_topic, sensor_qos)
        self.nav_pub = self.create_publisher(LaserScan, self.nav_scan_topic, sensor_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        mapping_enable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.enable_sub = self.create_subscription(
            Bool, self.mapping_enable_topic, self._enable_callback, mapping_enable_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_in_topic, self._scan_callback, sensor_qos)

        self._last_mapping_publish_time = 0.0
        self._last_nav_publish_time = 0.0
        self._scan_in_count = 0
        self._mapping_out_count = 0
        self._nav_out_count = 0
        self._last_scan_stamp = None
        self._last_scan_wall_time = 0.0

        self.create_timer(1.0 / max(0.1, self.status_rate_hz), self._publish_status)
        self.get_logger().info(
            f"scan gate ready: {self.scan_in_topic} -> "
            f"mapping={self.mapping_scan_topic} enabled={self.mapping_enabled}, "
            f"nav={self.nav_scan_topic}; rates mapping<={self.mapping_max_rate_hz:.2f}Hz "
            f"nav<={self.nav_max_rate_hz:.2f}Hz restamp={self.restamp_scans}")

    def _enable_callback(self, msg: Bool) -> None:
        self.mapping_enabled = bool(msg.data)
        self.get_logger().warn(
            f"mapping scan output {'enabled' if self.mapping_enabled else 'paused'}")
        self._publish_status()

    def _make_output_scan(self, msg: LaserScan) -> LaserScan:
        if not self.restamp_scans:
            return msg
        out = copy.deepcopy(msg)
        out.header.stamp = self.get_clock().now().to_msg()
        return out

    def _rate_allowed(self, last_time: float, max_rate_hz: float) -> bool:
        if max_rate_hz <= 0.0:
            return True
        return (time.monotonic() - last_time) >= (1.0 / max_rate_hz)

    def _scan_callback(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan_in_count += 1
        self._last_scan_stamp = msg.header.stamp
        self._last_scan_wall_time = now

        if self.nav_publish_enabled and self._rate_allowed(self._last_nav_publish_time, self.nav_max_rate_hz):
            self.nav_pub.publish(self._make_output_scan(msg))
            self._nav_out_count += 1
            self._last_nav_publish_time = now

        if self.mapping_enabled and self._rate_allowed(self._last_mapping_publish_time, self.mapping_max_rate_hz):
            self.mapping_pub.publish(self._make_output_scan(msg))
            self._mapping_out_count += 1
            self._last_mapping_publish_time = now

    def _publish_status(self) -> None:
        msg = String()
        age = None
        if self._last_scan_wall_time > 0.0:
            age = time.monotonic() - self._last_scan_wall_time
        msg.data = (
            f"mapping_enabled={str(self.mapping_enabled).lower()} "
            f"scan_in={self._scan_in_count} mapping_out={self._mapping_out_count} "
            f"nav_out={self._nav_out_count} restamp={str(self.restamp_scans).lower()} last_scan_age_sec={age if age is not None else 'none'}"
        )
        self.status_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScanGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
