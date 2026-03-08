#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    import mujoco
    from mujoco import viewer as mj_viewer
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        'The mujoco Python package is required for mujoco_bridge_node. '
        'Install it in the same environment as your ROS 2 workspace.'
    ) from exc


ACTUATOR_NAMES = [
    'v_fl_rightdown',
    'v_fr_rightdown',
    'v_lm_rightdown',
    'v_rm_rightdown',
    'v_bl_rightdown',
    'v_br_rightdown',
]

JOINT_NAMES = [
    'fl_rightdown',
    'fr_rightdown',
    'lm_rightdown',
    'rm_rightdown',
    'bl_rightdown',
    'br_rightdown',
]


@dataclass
class JointHandles:
    actuator_ids: Dict[str, int]
    qpos_adr: Dict[str, int]
    qvel_adr: Dict[str, int]


class MujocoBridgeNode(Node):
    """Bridge MuJoCo to ROS 2 topics needed by ros2_control and Nav2.

    This node does four jobs:
      1. step the MuJoCo model,
      2. apply six joint velocity commands,
      3. publish joint states, odom, IMU, RGB, depth, point cloud, TF, and /clock,
      4. make the simulation usable by a topic-based ros2_control system.

    Assumptions for the MJCF:
      - body name: base_link
      - camera name: rgbd_cam
      - site name: imu_site
      - sensors named: imu_accel, imu_gyro, imu_quat
      - six velocity actuators named in ACTUATOR_NAMES above
    """

    def __init__(self) -> None:
        super().__init__('mujoco_bridge_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_hz', 400.0)
        self.declare_parameter('render_hz', 20.0)
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('camera_name', 'rgbd_cam')
        self.declare_parameter('base_body', 'base_link')
        self.declare_parameter('imu_site', 'imu_site')
        self.declare_parameter('imu_accel_sensor', 'imu_accel')
        self.declare_parameter('imu_gyro_sensor', 'imu_gyro')
        self.declare_parameter('imu_quat_sensor', 'imu_quat')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('viewer', True)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fovy_deg', 69.0)

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            raise RuntimeError('model_path parameter must point to your MuJoCo scene.xml')

        self.sim_hz = float(self.get_parameter('sim_hz').value)
        self.render_hz = float(self.get_parameter('render_hz').value)
        self.point_stride = int(self.get_parameter('point_stride').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.base_body_name = str(self.get_parameter('base_body').value)
        self.imu_accel_sensor_name = str(self.get_parameter('imu_accel_sensor').value)
        self.imu_gyro_sensor_name = str(self.get_parameter('imu_gyro_sensor').value)
        self.imu_quat_sensor_name = str(self.get_parameter('imu_quat_sensor').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.camera_optical_frame = str(self.get_parameter('camera_optical_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.viewer_enabled = bool(self.get_parameter('viewer').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fovy_deg = float(self.get_parameter('camera_fovy_deg').value)

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, self.camera_height, self.camera_width)
        self.viewer = None

        if self.viewer_enabled:
            try:
                self.viewer = mj_viewer.launch_passive(
                    self.model,
                    self.data,
                    show_left_ui=True,
                    show_right_ui=True,
                )
            except Exception as exc:
                raise RuntimeError(
                    'Failed to open MuJoCo passive viewer. On Linux, ensure a desktop session / DISPLAY is available. '
                    'On macOS, launch through mjpython. You can also set viewer:=false to run headless.'
                ) from exc
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.handles = self._resolve_joint_handles()
        self.base_body_id = self._require_body(self.base_body_name)
        self.imu_sensor_ids = {
            'accel': self._require_sensor(self.imu_accel_sensor_name),
            'gyro': self._require_sensor(self.imu_gyro_sensor_name),
            'quat': self._require_sensor(self.imu_quat_sensor_name),
        }

        self.last_command: Dict[str, float] = {joint: 0.0 for joint in JOINT_NAMES}

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/topic_based_joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)

        self.command_sub = self.create_subscription(
            JointState,
            '/topic_based_joint_commands',
            self._joint_command_cb,
            10,
        )

        self.sim_timer = self.create_timer(1.0 / self.sim_hz, self._step_once)
        self.render_timer = self.create_timer(1.0 / self.render_hz, self._publish_render_products)

        self.get_logger().info(f'Loaded MuJoCo model from {model_path}')

    def _require_sensor(self, name: str) -> int:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            raise RuntimeError(f'MuJoCo sensor {name!r} was not found. Add it to robot.xml.')
        return sensor_id

    def _require_body(self, name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise RuntimeError(f'MuJoCo body {name!r} was not found.')
        return body_id

    def _resolve_joint_handles(self) -> JointHandles:
        actuator_ids: Dict[str, int] = {}
        qpos_adr: Dict[str, int] = {}
        qvel_adr: Dict[str, int] = {}

        for actuator_name, joint_name in zip(ACTUATOR_NAMES, JOINT_NAMES):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if actuator_id < 0:
                raise RuntimeError(f'Actuator {actuator_name!r} not found in MJCF')
            if joint_id < 0:
                raise RuntimeError(f'Joint {joint_name!r} not found in MJCF')

            actuator_ids[joint_name] = actuator_id
            qpos_adr[joint_name] = int(self.model.jnt_qposadr[joint_id])
            qvel_adr[joint_name] = int(self.model.jnt_dofadr[joint_id])

        return JointHandles(actuator_ids=actuator_ids, qpos_adr=qpos_adr, qvel_adr=qvel_adr)

    def _joint_command_cb(self, msg: JointState) -> None:
        # Topic-based ros2_control uses JointState messages.
        # For a velocity command interface, the command values are expected in msg.velocity.
        if not msg.name:
            return

        velocity_values: List[float]
        if msg.velocity:
            velocity_values = list(msg.velocity)
        elif msg.position:
            velocity_values = list(msg.position)
            self.get_logger().warn(
                'Received JointState command without velocity field; falling back to position array. '
                'If using velocity_controllers, verify your topic_based_hardware_interfaces version.'
            )
        else:
            return

        for joint_name, cmd in zip(msg.name, velocity_values):
            if joint_name in self.last_command:
                self.last_command[joint_name] = float(cmd)

    def _step_once(self) -> None:
        for joint_name, actuator_id in self.handles.actuator_ids.items():
            self.data.ctrl[actuator_id] = self.last_command[joint_name]

        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

        stamp = self.get_clock().now().to_msg()
        self._publish_clock()
        self._publish_joint_states(stamp)
        self._publish_imu(stamp)
        self._publish_odom_and_tf(stamp)

    def _publish_clock(self) -> None:
        msg = Clock()
        seconds = float(self.data.time)
        whole = int(math.floor(seconds))
        msg.clock.sec = whole
        msg.clock.nanosec = int((seconds - whole) * 1e9)
        self.clock_pub.publish(msg)

    def _publish_joint_states(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(self.data.qpos[self.handles.qpos_adr[name]]) for name in JOINT_NAMES]
        msg.velocity = [float(self.data.qvel[self.handles.qvel_adr[name]]) for name in JOINT_NAMES]
        msg.effort = [0.0 for _ in JOINT_NAMES]
        self.joint_state_pub.publish(msg)

    def _sensor_slice(self, sensor_id: int) -> np.ndarray:
        adr = int(self.model.sensor_adr[sensor_id])
        dim = int(self.model.sensor_dim[sensor_id])
        return np.array(self.data.sensordata[adr: adr + dim], dtype=np.float64)

    def _publish_imu(self, stamp) -> None:
        accel = self._sensor_slice(self.imu_sensor_ids['accel'])
        gyro = self._sensor_slice(self.imu_sensor_ids['gyro'])
        quat = self._sensor_slice(self.imu_sensor_ids['quat'])

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.orientation = Quaternion(x=float(quat[1]), y=float(quat[2]), z=float(quat[3]), w=float(quat[0]))
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        self.imu_pub.publish(msg)

    def _publish_odom_and_tf(self, stamp) -> None:
        xpos = np.array(self.data.xpos[self.base_body_id], dtype=np.float64)
        xquat = np.array(self.data.xquat[self.base_body_id], dtype=np.float64)

        vel = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.base_body_id,
            vel,
            0,
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.frame_id
        odom.pose.pose.position.x = float(xpos[0])
        odom.pose.pose.position.y = float(xpos[1])
        odom.pose.pose.position.z = float(xpos[2])
        odom.pose.pose.orientation = Quaternion(
            x=float(xquat[1]), y=float(xquat[2]), z=float(xquat[3]), w=float(xquat[0])
        )
        odom.twist.twist.linear.x = float(vel[3])
        odom.twist.twist.linear.y = float(vel[4])
        odom.twist.twist.linear.z = float(vel[5])
        odom.twist.twist.angular.x = float(vel[0])
        odom.twist.twist.angular.y = float(vel[1])
        odom.twist.twist.angular.z = float(vel[2])
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.frame_id
            tf.transform.translation.x = float(xpos[0])
            tf.transform.translation.y = float(xpos[1])
            tf.transform.translation.z = float(xpos[2])
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _camera_info_msg(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_optical_frame
        info.width = self.camera_width
        info.height = self.camera_height

        fy = self.camera_height / (2.0 * math.tan(math.radians(self.camera_fovy_deg) / 2.0))
        fx = fy
        cx = (self.camera_width - 1.0) / 2.0
        cy = (self.camera_height - 1.0) / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info

    def _publish_render_products(self) -> None:
        stamp = self.get_clock().now().to_msg()

        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()

        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()

        rgb_msg = self.bridge.cv2_to_imgmsg(np.asarray(rgb), encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.camera_optical_frame
        self.rgb_pub.publish(rgb_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(np.asarray(depth, dtype=np.float32), encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.camera_optical_frame
        self.depth_pub.publish(depth_msg)

        cam_info = self._camera_info_msg(stamp)
        self.camera_info_pub.publish(cam_info)
        self.cloud_pub.publish(self._depth_to_cloud(depth, cam_info, stamp))

    def _depth_to_cloud(self, depth: np.ndarray, cam_info: CameraInfo, stamp) -> PointCloud2:
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]

        step = max(1, self.point_stride)
        rows = np.arange(0, depth.shape[0], step)
        cols = np.arange(0, depth.shape[1], step)
        uu, vv = np.meshgrid(cols, rows)
        zz = depth[::step, ::step].astype(np.float32)

        valid = np.isfinite(zz) & (zz > 0.0)
        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        zz = zz[valid].astype(np.float32)

        xx = (uu - cx) * zz / fx
        yy = (vv - cy) * zz / fy
        points = np.stack((xx, yy, zz), axis=1).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_optical_frame
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        return msg


    def destroy_node(self) -> bool:
        if getattr(self, 'viewer', None) is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MujocoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
