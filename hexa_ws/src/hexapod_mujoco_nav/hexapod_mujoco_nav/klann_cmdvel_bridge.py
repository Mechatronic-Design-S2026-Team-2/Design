#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    'fl_rightdown',
    'fr_rightdown',
    'lm_rightdown',
    'rm_rightdown',
    'bl_rightdown',
    'br_rightdown',
]


@dataclass
class SideCommand:
    left: float
    right: float


class KlannCmdVelBridge(Node):
    """Map body-level cmd_vel into six Klann drive velocities.

    Two output modes are supported:
      1. Float64MultiArray for JointGroupVelocityController
      2. JointState velocity commands for topic_based_hardware_interfaces / direct MuJoCo bridge
    """

    def __init__(self) -> None:
        super().__init__('klann_cmdvel_bridge')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('joint_command_topic', '/topic_based_joint_commands')
        self.declare_parameter('forward_gain', 4.0)
        self.declare_parameter('yaw_gain', 2.0)
        self.declare_parameter('max_leg_speed', 8.0)
        self.declare_parameter('use_stamped_cmd', True)
        self.declare_parameter('publish_joint_state_commands', True)

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        joint_command_topic = str(self.get_parameter('joint_command_topic').value)
        self.forward_gain = float(self.get_parameter('forward_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.max_leg_speed = float(self.get_parameter('max_leg_speed').value)
        self.publish_joint_state_commands = bool(self.get_parameter('publish_joint_state_commands').value)

        self.command_pub_f64 = None
        self.command_pub_js = None
        if self.publish_joint_state_commands:
            self.command_pub_js = self.create_publisher(JointState, joint_command_topic, 10)
        else:
            self.command_pub_f64 = self.create_publisher(Float64MultiArray, joint_command_topic, 10)

        self.cmd_sub = self.create_subscription(TwistStamped, cmd_topic, self._cmd_cb, 10)

        out_type = 'JointState' if self.publish_joint_state_commands else 'Float64MultiArray'
        self.get_logger().info(
            f'Listening on {cmd_topic} and publishing {out_type} leg commands on {joint_command_topic}'
        )

    def _clamp(self, value: float) -> float:
        return max(-self.max_leg_speed, min(self.max_leg_speed, value))

    def _mix(self, vx: float, wz: float) -> SideCommand:
        left = self._clamp(self.forward_gain * vx - self.yaw_gain * wz)
        right = self._clamp(self.forward_gain * vx + self.yaw_gain * wz)
        return SideCommand(left=left, right=right)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        vx = float(msg.twist.linear.x)
        wz = float(msg.twist.angular.z)
        mixed = self._mix(vx, wz)
        cmd = [mixed.left, mixed.right, mixed.left, mixed.right, mixed.left, mixed.right]

        if self.publish_joint_state_commands:
            out = JointState()
            out.name = list(JOINT_NAMES)
            out.velocity = cmd
            self.command_pub_js.publish(out)
        else:
            out = Float64MultiArray()
            out.data = cmd
            self.command_pub_f64.publish(out)


def main() -> None:
    rclpy.init()
    node = KlannCmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
