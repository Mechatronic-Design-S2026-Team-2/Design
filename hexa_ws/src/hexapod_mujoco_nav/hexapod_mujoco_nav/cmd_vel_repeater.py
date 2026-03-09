#!/usr/bin/env python3

from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node


class CmdVelRepeater(Node):
    """
    Repeat the most recent TwistStamped command at a fixed rate.

    Purpose:
    - Accept bursty keyboard teleop input on /cmd_vel_key
    - Republish it continuously on /cmd_vel for the gait controller
    - After a timeout, publish zero velocity

    Typical use:
      teleop_twist_keyboard -> /cmd_vel_key
      cmd_vel_repeater      -> /cmd_vel
      gait_phase_controller <- /cmd_vel
    """

    def __init__(self) -> None:
        super().__init__('cmd_vel_repeater')

        self.declare_parameter('input_topic', '/cmd_vel_key')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('hold_timeout_sec', 0.20)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('force_zero_on_start', True)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.hold_timeout_sec = float(self.get_parameter('hold_timeout_sec').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.force_zero_on_start = bool(self.get_parameter('force_zero_on_start').value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')
        if self.hold_timeout_sec < 0.0:
            raise ValueError('hold_timeout_sec must be >= 0')

        self.last_cmd_msg: Optional[TwistStamped] = None
        self.last_cmd_time = None

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.input_topic,
            self._cmd_cb,
            20,
        )
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            20,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._timer_cb,
        )

        self.add_on_set_parameters_callback(self._param_cb)

        self.get_logger().info(
            f'Repeating {self.input_topic} -> {self.output_topic} '
            f'at {self.publish_rate_hz:.1f} Hz, hold_timeout_sec={self.hold_timeout_sec:.2f}'
        )

        if self.force_zero_on_start:
            self._publish_zero()

    def _param_cb(self, params):
        try:
            for p in params:
                if p.name == 'publish_rate_hz':
                    value = float(p.value)
                    if value <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason='publish_rate_hz must be > 0',
                        )
                elif p.name == 'hold_timeout_sec':
                    value = float(p.value)
                    if value < 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason='hold_timeout_sec must be >= 0',
                        )

            timer_needs_reset = False

            for p in params:
                if p.name == 'input_topic':
                    self.input_topic = str(p.value)
                elif p.name == 'output_topic':
                    self.output_topic = str(p.value)
                elif p.name == 'publish_rate_hz':
                    self.publish_rate_hz = float(p.value)
                    timer_needs_reset = True
                elif p.name == 'hold_timeout_sec':
                    self.hold_timeout_sec = float(p.value)
                elif p.name == 'frame_id':
                    self.frame_id = str(p.value)
                elif p.name == 'force_zero_on_start':
                    self.force_zero_on_start = bool(p.value)

            if timer_needs_reset:
                self.timer.cancel()
                self.timer = self.create_timer(
                    1.0 / self.publish_rate_hz,
                    self._timer_cb,
                )

            return SetParametersResult(successful=True)

        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self.last_cmd_msg = deepcopy(msg)

        if not self.last_cmd_msg.header.frame_id:
            self.last_cmd_msg.header.frame_id = self.frame_id

        self.last_cmd_time = self.get_clock().now()

    def _make_zero_msg(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        return msg

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(self._make_zero_msg())

    def _timer_cb(self) -> None:
        now = self.get_clock().now()

        if self.last_cmd_msg is None or self.last_cmd_time is None:
            self._publish_zero()
            return

        age = now - self.last_cmd_time
        if age > Duration(seconds=self.hold_timeout_sec):
            self._publish_zero()
            return

        out = deepcopy(self.last_cmd_msg)
        out.header.stamp = now.to_msg()
        if not out.header.frame_id:
            out.header.frame_id = self.frame_id

        self.cmd_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRepeater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
