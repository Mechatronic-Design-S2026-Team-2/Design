#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String


def wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GaitPhaseController(Node):
    def __init__(self) -> None:
        super().__init__('gait_phase_controller')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('joint_state_topic', '/topic_based_joint_states')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('joint_command_topic', '/topic_based_joint_commands')

        self.declare_parameter('control_hz', 200.0)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('cmd_deadband_linear_x', 0.01)
        self.declare_parameter('cmd_deadband_angular_z', 0.01)

        self.declare_parameter(
            'joint_names',
            ['fl_rightdown', 'fr_rightdown', 'lm_rightdown',
             'rm_rightdown', 'bl_rightdown', 'br_rightdown']
        )
        self.declare_parameter('joint_direction', [1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
        self.declare_parameter('joint_zero_offsets', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('forced_gait', 'tripod')
        self.declare_parameter('wave_speed_threshold', 0.08)

        self.declare_parameter('omega_gain_linear_x', 4.0)
        self.declare_parameter('omega_min_moving', 0.8)

        # New: dedicated yaw cadence
        self.declare_parameter('omega_gain_angular_z', 3.0)
        self.declare_parameter('omega_min_turning', 1.0)
        self.declare_parameter('yaw_curve_scale', 1.0)

        self.declare_parameter('phase_kp', 3.0)
        self.declare_parameter('vel_damping', 0.4)
        self.declare_parameter('sync_gain', 2.0)
        self.declare_parameter('max_leg_speed', 8.0)

        # Stability/sync clamp only, no longer used to limit yaw itself
        self.declare_parameter('max_stability_bias', 0.5)

        self.declare_parameter('roll_gain', 0.0)
        self.declare_parameter('pitch_gain', 0.0)
        self.declare_parameter('roll_rate_gain', 0.0)
        self.declare_parameter('pitch_rate_gain', 0.0)

        self.declare_parameter('enable_contact_feedback', False)
        self.declare_parameter('contact_gain', 0.0)

        self._load_params()

        self._joint_pos_raw: Dict[str, float] = {}
        self._joint_vel_raw: Dict[str, float] = {}

        self._imu_roll = 0.0
        self._imu_pitch = 0.0
        self._imu_roll_rate = 0.0
        self._imu_pitch_rate = 0.0

        self._last_cmd_vx = 0.0
        self._last_cmd_wz = 0.0
        self._last_cmd_time_sec: Optional[float] = None

        self._phase_des: Optional[List[float]] = None
        self._stand_hold_phase: Optional[List[float]] = None
        self._last_mode = 'stand'

        self._last_update_sec = self._now_sec()

        self._cmd_sub = self.create_subscription(
            TwistStamped, self.cmd_topic, self._cmd_cb, 20
        )
        self._joint_sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, 50
        )
        self._imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, 50
        )

        self._cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 20)
        self._mode_pub = self.create_publisher(String, '/gait_mode', 10)
        self._debug_pub = self.create_publisher(Float32MultiArray, '/gait_phase_debug', 10)

        self.add_on_set_parameters_callback(self._param_cb)

        self._timer = self.create_timer(1.0 / max(self.control_hz, 1.0), self._update)

        self.get_logger().info(
            f'Listening on {self.cmd_topic}, {self.joint_state_topic}, {self.imu_topic}; '
            f'publishing JointState velocity commands on {self.joint_command_topic}'
        )

    def _load_params(self) -> None:
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.joint_command_topic = self.get_parameter('joint_command_topic').value

        self.control_hz = float(self.get_parameter('control_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.cmd_deadband_linear_x = float(self.get_parameter('cmd_deadband_linear_x').value)
        self.cmd_deadband_angular_z = float(self.get_parameter('cmd_deadband_angular_z').value)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.joint_direction = [float(v) for v in self.get_parameter('joint_direction').value]
        self.joint_zero_offsets = [float(v) for v in self.get_parameter('joint_zero_offsets').value]

        self.forced_gait = str(self.get_parameter('forced_gait').value)
        self.wave_speed_threshold = float(self.get_parameter('wave_speed_threshold').value)

        self.omega_gain_linear_x = float(self.get_parameter('omega_gain_linear_x').value)
        self.omega_min_moving = float(self.get_parameter('omega_min_moving').value)

        self.omega_gain_angular_z = float(self.get_parameter('omega_gain_angular_z').value)
        self.omega_min_turning = float(self.get_parameter('omega_min_turning').value)
        self.yaw_curve_scale = float(self.get_parameter('yaw_curve_scale').value)

        self.phase_kp = float(self.get_parameter('phase_kp').value)
        self.vel_damping = float(self.get_parameter('vel_damping').value)
        self.sync_gain = float(self.get_parameter('sync_gain').value)
        self.max_leg_speed = float(self.get_parameter('max_leg_speed').value)
        self.max_stability_bias = float(self.get_parameter('max_stability_bias').value)

        self.roll_gain = float(self.get_parameter('roll_gain').value)
        self.pitch_gain = float(self.get_parameter('pitch_gain').value)
        self.roll_rate_gain = float(self.get_parameter('roll_rate_gain').value)
        self.pitch_rate_gain = float(self.get_parameter('pitch_rate_gain').value)

        self.enable_contact_feedback = bool(self.get_parameter('enable_contact_feedback').value)
        self.contact_gain = float(self.get_parameter('contact_gain').value)

        if len(self.joint_names) != 6 or len(self.joint_direction) != 6 or len(self.joint_zero_offsets) != 6:
            raise ValueError('joint_names, joint_direction, joint_zero_offsets must all have length 6')

    def _param_cb(self, params) -> SetParametersResult:
        try:
            for p in params:
                if p.name == 'cmd_timeout_sec':
                    self.cmd_timeout_sec = float(p.value)
                elif p.name == 'cmd_deadband_linear_x':
                    self.cmd_deadband_linear_x = float(p.value)
                elif p.name == 'cmd_deadband_angular_z':
                    self.cmd_deadband_angular_z = float(p.value)
                elif p.name == 'forced_gait':
                    self.forced_gait = str(p.value)
                elif p.name == 'wave_speed_threshold':
                    self.wave_speed_threshold = float(p.value)
                elif p.name == 'omega_gain_linear_x':
                    self.omega_gain_linear_x = float(p.value)
                elif p.name == 'omega_min_moving':
                    self.omega_min_moving = float(p.value)
                elif p.name == 'omega_gain_angular_z':
                    self.omega_gain_angular_z = float(p.value)
                elif p.name == 'omega_min_turning':
                    self.omega_min_turning = float(p.value)
                elif p.name == 'yaw_curve_scale':
                    self.yaw_curve_scale = float(p.value)
                elif p.name == 'phase_kp':
                    self.phase_kp = float(p.value)
                elif p.name == 'vel_damping':
                    self.vel_damping = float(p.value)
                elif p.name == 'sync_gain':
                    self.sync_gain = float(p.value)
                elif p.name == 'max_leg_speed':
                    self.max_leg_speed = float(p.value)
                elif p.name == 'max_stability_bias':
                    self.max_stability_bias = float(p.value)
                elif p.name == 'roll_gain':
                    self.roll_gain = float(p.value)
                elif p.name == 'pitch_gain':
                    self.pitch_gain = float(p.value)
                elif p.name == 'roll_rate_gain':
                    self.roll_rate_gain = float(p.value)
                elif p.name == 'pitch_rate_gain':
                    self.pitch_rate_gain = float(p.value)
                elif p.name == 'enable_contact_feedback':
                    self.enable_contact_feedback = bool(p.value)
                elif p.name == 'contact_gain':
                    self.contact_gain = float(p.value)
                elif p.name == 'joint_direction':
                    vals = [float(v) for v in p.value]
                    if len(vals) != 6:
                        return SetParametersResult(successful=False, reason='joint_direction must have length 6')
                    self.joint_direction = vals
                elif p.name == 'joint_zero_offsets':
                    vals = [float(v) for v in p.value]
                    if len(vals) != 6:
                        return SetParametersResult(successful=False, reason='joint_zero_offsets must have length 6')
                    self.joint_zero_offsets = vals
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._last_cmd_vx = float(msg.twist.linear.x)
        self._last_cmd_wz = float(msg.twist.angular.z)
        self._last_cmd_time_sec = self._now_sec()

    def _joint_state_cb(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos_raw[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self._joint_vel_raw[name] = float(msg.velocity[i])

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._imu_roll = roll
        self._imu_pitch = pitch
        self._imu_roll_rate = float(msg.angular_velocity.x)
        self._imu_pitch_rate = float(msg.angular_velocity.y)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _get_joint_virtual_state(self) -> Optional[Tuple[List[float], List[float]]]:
        q_virtual: List[float] = []
        qd_virtual: List[float] = []
        for i, name in enumerate(self.joint_names):
            if name not in self._joint_pos_raw:
                return None
            q_raw = self._joint_pos_raw.get(name, 0.0)
            qd_raw = self._joint_vel_raw.get(name, 0.0)
            qv = self.joint_direction[i] * (q_raw - self.joint_zero_offsets[i])
            qdv = self.joint_direction[i] * qd_raw
            q_virtual.append(wrap_pi(qv))
            qd_virtual.append(qdv)
        return q_virtual, qd_virtual

    def _cmd_is_fresh(self) -> bool:
        return self._last_cmd_time_sec is not None and (self._now_sec() - self._last_cmd_time_sec) <= self.cmd_timeout_sec

    def _latest_cmd(self) -> Tuple[float, float]:
        if not self._cmd_is_fresh():
            return 0.0, 0.0
        return self._last_cmd_vx, self._last_cmd_wz

    def _select_mode(self, vx: float, wz: float) -> str:
        if self.forced_gait in {'stand', 'tripod', 'wave'}:
            if self.forced_gait == 'stand':
                return 'stand'
            if abs(vx) < self.cmd_deadband_linear_x and abs(wz) < self.cmd_deadband_angular_z:
                return 'stand'
            return self.forced_gait

        if abs(vx) < self.cmd_deadband_linear_x and abs(wz) < self.cmd_deadband_angular_z:
            return 'stand'
        if abs(vx) < self.wave_speed_threshold:
            return 'wave'
        return 'tripod'

    def _gait_template(self, mode: str) -> List[float]:
        if mode == 'tripod':
            return [0.0, math.pi, math.pi, 0.0, 0.0, math.pi]
        if mode == 'wave':
            return [0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, -2.0 * math.pi / 3.0, -math.pi / 3.0]
        return [0.0] * 6

    def _omega_base(self, vx: float) -> float:
        if abs(vx) < self.cmd_deadband_linear_x:
            return 0.0
        mag = max(self.omega_min_moving, abs(vx) * self.omega_gain_linear_x)
        return math.copysign(mag, vx)

    def _turn_rate(self, wz: float) -> float:
        if abs(wz) < self.cmd_deadband_angular_z:
            return 0.0
        mag = max(self.omega_min_turning, abs(wz) * self.omega_gain_angular_z)
        return math.copysign(mag, wz)

    def _sync_biases(self, mode: str) -> List[float]:
        if self._phase_des is None:
            return [0.0] * 6

        template = self._gait_template(mode)
        out = [0.0] * 6
        for i in range(6):
            s = 0.0
            for j in range(6):
                if i == j:
                    continue
                desired_diff = wrap_pi(template[j] - template[i])
                actual_diff = wrap_pi(self._phase_des[j] - self._phase_des[i])
                s += math.sin(wrap_pi(actual_diff - desired_diff))
            out[i] = self.sync_gain * s / 5.0
        return out

    def _stability_biases(self) -> List[float]:
        side_sign = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
        fore_sign = [1.0, 1.0, 0.0, 0.0, -1.0, -1.0]

        roll_term = self.roll_gain * self._imu_roll + self.roll_rate_gain * self._imu_roll_rate
        pitch_term = self.pitch_gain * self._imu_pitch + self.pitch_rate_gain * self._imu_pitch_rate

        out = [0.0] * 6
        for i in range(6):
            out[i] = side_sign[i] * roll_term + fore_sign[i] * pitch_term
        return out

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _publish_debug(self, mode, omega_base, turn_rate, q_virtual, qd_virtual, q_des, qd_des, cmd_raw) -> None:
        mode_id = {'stand': 0.0, 'tripod': 1.0, 'wave': 2.0}.get(mode, -1.0)
        err = [wrap_pi(q_des[i] - q_virtual[i]) for i in range(6)]
        msg = Float32MultiArray()
        msg.data = (
            [mode_id, omega_base, turn_rate, self._imu_roll, self._imu_pitch,
             self._imu_roll_rate, self._imu_pitch_rate]
            + q_virtual + qd_virtual + q_des + qd_des + err + cmd_raw
        )
        self._debug_pub.publish(msg)

    def _update(self) -> None:
        state = self._get_joint_virtual_state()
        if state is None:
            return

        q_virtual, qd_virtual = state

        now_sec = self._now_sec()
        dt = clamp(now_sec - self._last_update_sec, 1e-4, 0.1)
        self._last_update_sec = now_sec

        vx, wz = self._latest_cmd()
        mode = self._select_mode(vx, wz)

        if self._phase_des is None:
            self._phase_des = list(q_virtual)

        if mode == 'stand':
            if self._last_mode != 'stand' or self._stand_hold_phase is None:
                self._stand_hold_phase = list(q_virtual)
            q_des = list(self._stand_hold_phase)
            qd_des = [0.0] * 6
            self._phase_des = list(q_des)
            omega_base = 0.0
            turn_rate = 0.0
        else:
            if self._last_mode == 'stand':
                self._phase_des = list(q_virtual)

            omega_base = self._omega_base(vx)
            turn_rate = self._turn_rate(wz)

            # Differential phase-rate term in gait space
            yaw_term = [
                -self.yaw_curve_scale * turn_rate,
                +self.yaw_curve_scale * turn_rate,
                -self.yaw_curve_scale * turn_rate,
                +self.yaw_curve_scale * turn_rate,
                -self.yaw_curve_scale * turn_rate,
                +self.yaw_curve_scale * turn_rate,
            ]

            stab_bias = self._stability_biases()
            sync_bias = self._sync_biases(mode)

            q_des = [0.0] * 6
            qd_des = [0.0] * 6

            for i in range(6):
                small_bias = clamp(
                    stab_bias[i] + sync_bias[i],
                    -self.max_stability_bias,
                    self.max_stability_bias
                )

                qd_des[i] = omega_base + yaw_term[i] + small_bias
                qd_des[i] = clamp(qd_des[i], -self.max_leg_speed, self.max_leg_speed)

                self._phase_des[i] = wrap_pi(self._phase_des[i] + qd_des[i] * dt)
                q_des[i] = self._phase_des[i]

        cmd_raw = [0.0] * 6
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.joint_names)
        cmd_msg.velocity = [0.0] * 6

        for i in range(6):
            err = wrap_pi(q_des[i] - q_virtual[i])
            u_virtual = qd_des[i] + self.phase_kp * err - self.vel_damping * qd_virtual[i]
            u_raw = self.joint_direction[i] * u_virtual
            u_raw = clamp(u_raw, -self.max_leg_speed, self.max_leg_speed)
            cmd_raw[i] = u_raw
            cmd_msg.velocity[i] = u_raw

        self._cmd_pub.publish(cmd_msg)
        self._publish_mode(mode)
        self._publish_debug(mode, omega_base, turn_rate, q_virtual, qd_virtual, q_des, qd_des, cmd_raw)
        self._last_mode = mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitPhaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
