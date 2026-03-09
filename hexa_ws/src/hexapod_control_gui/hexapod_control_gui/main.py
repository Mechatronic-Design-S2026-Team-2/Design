\
#!/usr/bin/env python3
import sys
import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import BatteryState, Imu, Image
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray

# Custom message (build klann_msgs in your workspace)
try:
    from klann_msgs.msg import ForceSensorVoltages
    _HAVE_FORCE_MSG = True
except Exception:
    try:
        from hexapod_msgs.msg import ForceSensorVoltages  # fallback if you used the older package name
        _HAVE_FORCE_MSG = True
    except Exception:
        ForceSensorVoltages = None
        _HAVE_FORCE_MSG = False

# Qt
from PyQt5 import QtCore, QtGui, QtWidgets

# Optional plotting (recommended): sudo apt install python3-pyqtgraph
try:
    import pyqtgraph as pg
    _HAVE_PG = True
except Exception:
    pg = None
    _HAVE_PG = False

try:
    import numpy as np
except Exception:
    np = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


IMPORTANT_TOPICS = [
    "/camera/camera_info",
    "/camera/color/image_raw",
    "/camera/depth/image_rect_raw",
    "/camera/depth/points",
    "/clock",
    "/cmd_vel",
    "/cmd_vel_key",
    "/cmd_vel_nav",
    "/cmd_vel_teleop",
    "/diagnostics",
    "/force_sensor_voltage_millivolts",
    "/gait_mode",
    "/gait_phase_debug",
    "/imu/data",
    "/imu/data_raw",
    "/joint_states",
    "/leg_velocity_controller/commands",
    "/mode/operation",
    "/mode/rider_present",
    "/odom",
    "/power/battery_state",
    "/robot_description",
    "/safety/e_stop",
    "/tf",
    "/tf_static",
    "/topic_based_joint_commands",
    "/topic_based_joint_states",
    "/twist_mux/noop_lock",
]


def yaw_from_quat(q) -> float:
    # q is geometry_msgs/Quaternion
    # yaw = atan2(2(wz+xy), 1-2(y^2+z^2))
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def clip16(s: str) -> str:
    return (s or "")[:16]


def qimage_from_rgb(rgb8: bytes, w: int, h: int) -> QtGui.QImage:
    # rgb8 is packed RGB bytes
    img = QtGui.QImage(rgb8, w, h, 3*w, QtGui.QImage.Format_RGB888)
    return img.copy()


def qimage_from_gray(gray8: bytes, w: int, h: int) -> QtGui.QImage:
    img = QtGui.QImage(gray8, w, h, w, QtGui.QImage.Format_Grayscale8)
    return img.copy()


class GuiNode(Node):
    def __init__(self):
        super().__init__("hexapod_control_gui_node")

        # Parameters
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("battery_topic", "/power/battery_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("force_topic", "/force_sensor_voltage_millivolts")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_teleop")
        self.declare_parameter("estop_topic", "/safety/e_stop")
        self.declare_parameter("mode_topic", "/mode/operation")
        self.declare_parameter("rider_topic", "/mode/rider_present")
        self.declare_parameter("gait_mode_topic", "/gait_mode")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_vx", 0.25)
        self.declare_parameter("max_vy", 0.25)
        self.declare_parameter("max_wz", 0.9)

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.battery_topic = self.get_parameter("battery_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.diag_topic = self.get_parameter("diagnostics_topic").value
        self.force_topic = self.get_parameter("force_topic").value

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value
        self.mode_topic = self.get_parameter("mode_topic").value
        self.rider_topic = self.get_parameter("rider_topic").value
        self.gait_mode_topic = self.get_parameter("gait_mode_topic").value

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        # QoS for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._bridge = CvBridge() if CvBridge else None

        # State updated by callbacks
        self.last_battery: Optional[BatteryState] = None
        self.last_odom: Optional[Odometry] = None
        self.last_imu: Optional[Imu] = None
        self.last_diag: Optional[DiagnosticArray] = None
        self.last_mode: Optional[str] = None
        self.last_rider: Optional[bool] = None
        self.last_estop: Optional[bool] = None

        self.last_color_qimage: Optional[QtGui.QImage] = None
        self.last_depth_qimage: Optional[QtGui.QImage] = None
        self._color_fps = 0.0
        self._depth_fps = 0.0
        self._color_last_t = None
        self._depth_last_t = None
        self._depth_center_m: Optional[float] = None
        self._depth_min_m: Optional[float] = None

        # Subscriptions
        self.create_subscription(BatteryState, self.battery_topic, self._on_battery, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)

        if _HAVE_FORCE_MSG:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(ForceSensorVoltages, self.force_topic, self._on_force, qos)
        self.create_subscription(DiagnosticArray, self.diag_topic, self._on_diag, 10)
        self.create_subscription(String, self.mode_topic, self._on_mode, 10)
        self.create_subscription(Bool, self.rider_topic, self._on_rider, 10)
        self.create_subscription(Bool, self.estop_topic, self._on_estop, 10)

        self.create_subscription(Image, self.color_topic, self._on_color, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth, sensor_qos)

        # Publishers
        # ROS 2 does not allow the same topic name with multiple message types in one graph.
        # Therefore we must create exactly ONE publisher for cmd_vel, matching the existing topic type.
        #
        # Parameter `cmd_vel_type`:
        #   - 'auto' (default): inspect graph; if topic exists and is TwistStamped, use it; else Twist.
        #   - 'twist': force geometry_msgs/Twist
        #   - 'twiststamped': force geometry_msgs/TwistStamped (only valid if the topic is TwistStamped)
        self.declare_parameter("cmd_vel_type", "auto")
        self._cmd_vel_is_stamped = self._resolve_cmd_vel_is_stamped()

        if self._cmd_vel_is_stamped:
            self._cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self._estop_pub = self.create_publisher(Bool, self.estop_topic, 10)
        self._mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self._rider_pub = self.create_publisher(Bool, self.rider_topic, 10)

        # gait_mode can be published as std_msgs/String or std_msgs/Int32, but ROS 2 does not allow
        # multiple types on the same topic name. Therefore we publish exactly ONE type that matches
        # the existing topic type (or a forced param if the topic does not exist yet).
        #
        # Parameter `gait_mode_type`:
        #   - 'auto' (default): if /gait_mode exists and is Int32, publish Int32; else publish String
        #   - 'string': force String (if topic already exists as Int32, we will still match Int32)
        #   - 'int32': force Int32 (if topic already exists as String, we will still match String)
        self.declare_parameter("gait_mode_type", "auto")
        self._gait_is_int = self._resolve_gait_mode_is_int()
        self._gait_pub = self.create_publisher(Int32 if self._gait_is_int else String, self.gait_mode_topic, 10)
        self._gait_use_int = self._gait_is_int  # UI toggle can only reduce capability, never change topic type

        self._last_type_probe = 0.0

    # ----- callbacks -----

    def _resolve_cmd_vel_is_stamped(self) -> bool:
        # Returns True if we should publish TwistStamped on cmd_vel_topic.
        # If the topic already exists, match its type to avoid rclcpp type conflicts.
        forced = str(self.get_parameter("cmd_vel_type").value).strip().lower()
        if forced in ("twist", "unstamped"):
            return False
        if forced in ("twiststamped", "stamped"):
            return True

        # auto mode: inspect the graph
        try:
            for name, types in self.get_topic_names_and_types():
                if name == self.cmd_vel_topic:
                    # Types look like 'geometry_msgs/msg/Twist' or 'geometry_msgs/msg/TwistStamped'
                    if any("TwistStamped" in t for t in types):
                        return True
                    return False
        except Exception:
            pass

        # Default: Twist (most compatible with simple mux + micro-controllers)
        return False

    def _resolve_gait_mode_is_int(self) -> bool:
        forced = str(self.get_parameter("gait_mode_type").value).strip().lower()
        # Inspect existing topic type first, to avoid type conflicts.
        try:
            for name, types in self.get_topic_names_and_types():
                if name == self.gait_mode_topic:
                    if any("Int32" in t for t in types):
                        return True
                    return False
        except Exception:
            pass

        # Topic does not exist yet: honor forced preference.
        if forced in ("int32", "i32", "int"):
            return True
        return False
    def _on_battery(self, msg: BatteryState):
        self.last_battery = msg

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _on_imu(self, msg: Imu):
        self.last_imu = msg

    def _on_force(self, msg: ForceSensorVoltages) -> None:
        try:
            self.last_force = [int(x) for x in msg.voltage_millivolts[:6]]
        except Exception:
            self.last_force = None

    def _on_diag(self, msg: DiagnosticArray):
        self.last_diag = msg

    def _on_mode(self, msg: String):
        self.last_mode = (msg.data or "").strip()

    def _on_rider(self, msg: Bool):
        self.last_rider = bool(msg.data)

    def _on_estop(self, msg: Bool):
        self.last_estop = bool(msg.data)

    def _update_fps(self, last_t: Optional[float], now_t: float, alpha: float = 0.2) -> Tuple[Optional[float], float]:
        if last_t is None:
            return now_t, 0.0
        dt = max(1e-6, now_t - last_t)
        fps = 1.0 / dt
        return now_t, fps

    def _on_color(self, msg: Image):
        now = time.time()
        self._color_last_t, fps = self._update_fps(self._color_last_t, now)
        if fps > 0:
            self._color_fps = 0.8 * self._color_fps + 0.2 * fps if self._color_fps > 0 else fps

        try:
            if self._bridge:
                cv = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                if np is None:
                    return
                rgb = cv.tobytes()
                self.last_color_qimage = qimage_from_rgb(rgb, msg.width, msg.height)
                return
            # manual decode for rgb8/bgr8/mono8
            enc = (msg.encoding or "").lower()
            if enc == "rgb8":
                self.last_color_qimage = qimage_from_rgb(msg.data, msg.width, msg.height)
            elif enc == "bgr8" and np is not None:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                arr = arr[:, :, ::-1]  # BGR->RGB
                self.last_color_qimage = qimage_from_rgb(arr.tobytes(), msg.width, msg.height)
            elif enc == "mono8":
                self.last_color_qimage = qimage_from_gray(msg.data, msg.width, msg.height)
        except Exception:
            return

    def _on_depth(self, msg: Image):
        now = time.time()
        self._depth_last_t, fps = self._update_fps(self._depth_last_t, now)
        if fps > 0:
            self._depth_fps = 0.8 * self._depth_fps + 0.2 * fps if self._depth_fps > 0 else fps

        # Try to compute min/center depth (meters) if float32 or uint16
        try:
            enc = (msg.encoding or "").lower()
            if np is not None:
                if enc in ("32fc1",):
                    arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
                    center = float(arr[msg.height//2, msg.width//2])
                    valid = arr[np.isfinite(arr) & (arr > 0.05) & (arr < 10.0)]
                    self._depth_center_m = center if math.isfinite(center) else None
                    self._depth_min_m = float(valid.min()) if valid.size else None
                    # display: normalize to 8-bit
                    if valid.size:
                        vmax = float(np.percentile(valid, 95))
                        vmin = float(np.percentile(valid, 5))
                        denom = max(1e-6, vmax - vmin)
                        img8 = np.clip((arr - vmin) / denom * 255.0, 0, 255).astype(np.uint8)
                        self.last_depth_qimage = qimage_from_gray(img8.tobytes(), msg.width, msg.height)
                    return
                if enc in ("16uc1",):
                    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
                    # RealSense depth is typically in millimeters
                    center_mm = int(arr[msg.height//2, msg.width//2])
                    valid = arr[(arr > 50) & (arr < 10000)]
                    self._depth_center_m = (center_mm / 1000.0) if center_mm > 0 else None
                    self._depth_min_m = (int(valid.min()) / 1000.0) if valid.size else None
                    if valid.size:
                        vmax = int(np.percentile(valid, 95))
                        vmin = int(np.percentile(valid, 5))
                        denom = max(1, vmax - vmin)
                        img8 = np.clip((arr.astype(np.int32) - vmin) / denom * 255.0, 0, 255).astype(np.uint8)
                        self.last_depth_qimage = qimage_from_gray(img8.tobytes(), msg.width, msg.height)
                    return
        except Exception:
            pass

    # ----- publishing API -----
    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        vx = max(-self.max_vx, min(self.max_vx, vx))
        vy = max(-self.max_vy, min(self.max_vy, vy))
        wz = max(-self.max_wz, min(self.max_wz, wz))

        if self._cmd_vel_is_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(vx)
            msg.twist.linear.y = float(vy)
            msg.twist.angular.z = float(wz)
            self._cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(wz)
            self._cmd_pub.publish(msg)

    def publish_estop(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._estop_pub.publish(msg)

    def publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = str(mode)
        self._mode_pub.publish(msg)

    def publish_rider(self, present: bool) -> None:
        msg = Bool()
        msg.data = bool(present)
        self._rider_pub.publish(msg)

    def publish_gait_mode(self, value: str) -> None:
        # Publishes using the topic's resolved type (String or Int32).
        if self._gait_is_int and self._gait_use_int:
            try:
                v = int(value)
            except Exception:
                v = 0
            m = Int32()
            m.data = v
            self._gait_pub.publish(m)
        else:
            m = String()
            m.data = str(value)
            self._gait_pub.publish(m)

    def set_gait_mode_type(self, use_int: bool) -> None:
        self._gait_use_int = bool(use_int)


@dataclass
class KeyState:
    forward: bool = False
    back: bool = False
    left: bool = False
    right: bool = False
    yaw_left: bool = False
    yaw_right: bool = False


class CameraWidget(QtWidgets.QWidget):
    def __init__(self, title: str):
        super().__init__()
        self._title = QtWidgets.QLabel(title)
        self._title.setStyleSheet("font-weight:600;")
        self._img = QtWidgets.QLabel("no image")
        self._img.setAlignment(QtCore.Qt.AlignCenter)
        self._img.setMinimumSize(320, 240)
        self._img.setStyleSheet("background:#111; color:#aaa; border:1px solid #333;")
        self._meta = QtWidgets.QLabel("")
        self._meta.setStyleSheet("color:#aaa;")

        lay = QtWidgets.QVBoxLayout()
        lay.addWidget(self._title)
        lay.addWidget(self._img, 1)
        lay.addWidget(self._meta)
        self.setLayout(lay)

    def set_image(self, qimg: Optional[QtGui.QImage], meta: str = ""):
        if qimg is None:
            self._img.setText("no image")
            self._img.setPixmap(QtGui.QPixmap())
            self._meta.setText(meta)
            return
        pix = QtGui.QPixmap.fromImage(qimg)
        pix = pix.scaled(self._img.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self._img.setPixmap(pix)
        self._meta.setText(meta)

    def resizeEvent(self, ev):
        # Force redraw scaling on resize
        super().resizeEvent(ev)


class TimeseriesPlot(QtWidgets.QWidget):
    """Simple rolling plot using pyqtgraph, with a text fallback."""

    def __init__(self, title: str, series_names: List[str], history_len: int = 300, y_range=None):
        super().__init__()
        self._title = QtWidgets.QLabel(title)
        self._title.setStyleSheet("font-weight:600;")
        self._history_len = int(history_len)
        self._series_names = list(series_names)
        self._t = []
        self._y = [[] for _ in self._series_names]

        lay = QtWidgets.QVBoxLayout()
        lay.addWidget(self._title)

        if _HAVE_PG:
            self._plot = pg.PlotWidget()
            self._plot.showGrid(x=True, y=True, alpha=0.2)
            if y_range is not None:
                try:
                    self._plot.setYRange(float(y_range[0]), float(y_range[1]), padding=0.0)
                except Exception:
                    pass
            self._plot.addLegend()
            self._curves = []
            for name in self._series_names:
                c = self._plot.plot([], [], name=name)
                self._curves.append(c)
            lay.addWidget(self._plot, 1)
            self._fallback = None
        else:
            self._plot = None
            self._curves = None
            self._fallback = QtWidgets.QLabel("pyqtgraph not installed")
            self._fallback.setStyleSheet("color:#aaa;")
            lay.addWidget(self._fallback)

        self.setLayout(lay)

    def append(self, t: float, values: List[float]):
        if values is None or len(values) != len(self._series_names):
            return
        self._t.append(float(t))
        for i, v in enumerate(values):
            self._y[i].append(float(v))

        # trim
        if len(self._t) > self._history_len:
            self._t = self._t[-self._history_len:]
            for i in range(len(self._y)):
                self._y[i] = self._y[i][-self._history_len:]

        if self._plot is not None:
            # x as seconds relative to latest for readability
            t0 = self._t[0]
            xs = [ti - t0 for ti in self._t]
            for i, c in enumerate(self._curves):
                c.setData(xs, self._y[i])
        elif self._fallback is not None:
            # show latest values
            last = [f"{v:.2f}" for v in values]
            self._fallback.setText(", ".join(last)[:120])

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Hexapod Control GUI (ROS 2)")
        self.resize(1200, 800)

        self.keys = KeyState()
        self.teleop_enabled = True
        self.vx_scale = 0.15
        self.vy_scale = 0.15
        self.wz_scale = 0.5

        # Central tabs
        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)

        # --- Teleop tab ---
        teleop = QtWidgets.QWidget()
        tabs.addTab(teleop, "Teleop")

        self.chk_enable = QtWidgets.QCheckBox("Enable teleop publishing (/cmd_vel_teleop)")
        self.chk_enable.setChecked(True)
        self.chk_enable.stateChanged.connect(lambda s: setattr(self, "teleop_enabled", s == QtCore.Qt.Checked))

        self.lbl_cmd = QtWidgets.QLabel("cmd: vx=0 vy=0 wz=0")
        self.lbl_cmd.setStyleSheet("font-family: monospace;")

        self.btn_stop = QtWidgets.QPushButton("STOP (Space)")
        self.btn_stop.setStyleSheet("font-weight:700; background:#c0392b; color:white; padding:12px;")
        self.btn_stop.clicked.connect(self._stop)

        def slider_row(label: str, minv: int, maxv: int, initv: int):
            lab = QtWidgets.QLabel(label)
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setMinimum(minv); s.setMaximum(maxv); s.setValue(initv)
            val = QtWidgets.QLabel(str(initv))
            val.setFixedWidth(60)
            lay = QtWidgets.QHBoxLayout()
            lay.addWidget(lab); lay.addWidget(s, 1); lay.addWidget(val)
            w = QtWidgets.QWidget(); w.setLayout(lay)
            return w, s, val

        row1, s_vx, v_vx = slider_row("Max |vx| (cm/s)", 0, 50, int(self.vx_scale*100))
        row2, s_vy, v_vy = slider_row("Max |vy| (cm/s)", 0, 50, int(self.vy_scale*100))
        row3, s_wz, v_wz = slider_row("Max |wz| (deg/s)", 0, 180, int(self.wz_scale*180/math.pi))

        def on_vx(v): 
            self.vx_scale = v/100.0; v_vx.setText(str(v))
        def on_vy(v): 
            self.vy_scale = v/100.0; v_vy.setText(str(v))
        def on_wz(v):
            self.wz_scale = (v*math.pi/180.0); v_wz.setText(str(v))

        s_vx.valueChanged.connect(on_vx)
        s_vy.valueChanged.connect(on_vy)
        s_wz.valueChanged.connect(on_wz)

        hint = QtWidgets.QLabel("Keys: W/S vx, A/D vy, Q/E yaw, Space stop. (Click inside the window first.)")
        hint.setStyleSheet("color:#666;")

        tl = QtWidgets.QVBoxLayout()
        tl.addWidget(self.chk_enable)
        tl.addWidget(row1); tl.addWidget(row2); tl.addWidget(row3)
        tl.addWidget(self.btn_stop)
        tl.addWidget(self.lbl_cmd)
        tl.addStretch(1)
        tl.addWidget(hint)
        teleop.setLayout(tl)

        # --- Modes tab ---
        modes = QtWidgets.QWidget()
        tabs.addTab(modes, "Mode / Safety")

        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItems(["DIRECT", "AUTO_POSE", "AUTO_WAYPOINTS", "RETURN_HOME"])
        self.btn_set_mode = QtWidgets.QPushButton("Publish mode")
        self.btn_set_mode.clicked.connect(self._publish_mode)

        self.chk_rider = QtWidgets.QCheckBox("Rider present (/mode/rider_present)")
        self.chk_rider.stateChanged.connect(self._publish_rider)

        self.btn_estop = QtWidgets.QPushButton("E-STOP TOGGLE (/safety/e_stop)")
        self.btn_estop.setStyleSheet("font-weight:700; background:#8e44ad; color:white; padding:10px;")
        self.btn_estop.clicked.connect(self._toggle_estop)

        self.gait_edit = QtWidgets.QLineEdit()
        self.gait_edit.setPlaceholderText("gait_mode value (string or int)")
        self.chk_gait_int = QtWidgets.QCheckBox("Publish gait_mode as Int32")
        self.chk_gait_int.stateChanged.connect(lambda s: self.node.set_gait_mode_type(s == QtCore.Qt.Checked))
        self.btn_gait = QtWidgets.QPushButton("Publish /gait_mode")
        self.btn_gait.clicked.connect(self._publish_gait_mode)

        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel("Operation mode:"), 0, 0)
        grid.addWidget(self.combo_mode, 0, 1)
        grid.addWidget(self.btn_set_mode, 0, 2)
        grid.addWidget(self.chk_rider, 1, 0, 1, 2)
        grid.addWidget(self.btn_estop, 2, 0, 1, 3)
        grid.addWidget(QtWidgets.QLabel("Gait mode:"), 3, 0)
        grid.addWidget(self.gait_edit, 3, 1)
        grid.addWidget(self.btn_gait, 3, 2)
        grid.addWidget(self.chk_gait_int, 4, 0, 1, 2)
        grid.setRowStretch(5, 1)
        modes.setLayout(grid)

        # --- Cameras tab ---
        cams = QtWidgets.QWidget()
        tabs.addTab(cams, "Cameras")

        self.color_w = CameraWidget("Color")
        self.depth_w = CameraWidget("Depth")

        hl = QtWidgets.QHBoxLayout()
        hl.addWidget(self.color_w, 1)
        hl.addWidget(self.depth_w, 1)
        cams.setLayout(hl)

        # --- Status tab ---
        status = QtWidgets.QWidget()
        tabs.addTab(status, "Status / Topics")

        self.lbl_batt = QtWidgets.QLabel("Battery: --")
        self.lbl_odom = QtWidgets.QLabel("Odom: --")
        self.lbl_imu = QtWidgets.QLabel("IMU: --")
        self.lbl_diag = QtWidgets.QLabel("Diagnostics: --")
        self.lbl_force = QtWidgets.QLabel("Force: --")
        self.lbl_force.setStyleSheet("font-family: monospace;")

        # Plots
        self.force_plot = TimeseriesPlot("Force sensor voltages [mV]", [f"ch{i}" for i in range(6)], history_len=300, y_range=(0, 2500))
        self.imu_plot = TimeseriesPlot("IMU raw: ang_vel [rad/s] + lin_acc [m/s^2]", ["wx","wy","wz","ax","ay","az"], history_len=300)

        for l in (self.lbl_batt, self.lbl_odom, self.lbl_imu, self.lbl_diag):
            l.setStyleSheet("font-family: monospace;")

        self.topic_table = QtWidgets.QTableWidget()
        self.topic_table.setColumnCount(3)
        self.topic_table.setHorizontalHeaderLabels(["Topic", "Present", "Types"])
        self.topic_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.topic_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.topic_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.topic_table.setRowCount(len(IMPORTANT_TOPICS))
        for i, t in enumerate(IMPORTANT_TOPICS):
            self.topic_table.setItem(i, 0, QtWidgets.QTableWidgetItem(t))
            self.topic_table.setItem(i, 1, QtWidgets.QTableWidgetItem("no"))
            self.topic_table.setItem(i, 2, QtWidgets.QTableWidgetItem(""))

        v = QtWidgets.QVBoxLayout()
        v.addWidget(self.lbl_batt)
        v.addWidget(self.lbl_odom)
        v.addWidget(self.lbl_imu)
        v.addWidget(self.lbl_diag)
        v.addWidget(self.lbl_force)
        v.addWidget(self.force_plot, 1)
        v.addWidget(self.imu_plot, 1)
        v.addWidget(QtWidgets.QLabel("Topic presence (as discovered by ROS graph):"))
        v.addWidget(self.topic_table, 1)
        status.setLayout(v)

        # Timers: ROS spin + GUI refresh + cmd publishing
        self._spin_timer = QtCore.QTimer()
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(10)

        self._ui_timer = QtCore.QTimer()
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(100)

        self._cmd_timer = QtCore.QTimer()
        self._cmd_timer.timeout.connect(self._publish_cmd_from_keys)
        self._cmd_timer.start(int(1000.0 / max(1.0, self.node.publish_rate_hz)))

        self._last_graph_refresh = 0.0

        self.statusBar().showMessage("Ready")

    # ---- key handling ----
    def keyPressEvent(self, ev: QtGui.QKeyEvent):
        if ev.isAutoRepeat():
            return
        k = ev.key()
        if k == QtCore.Qt.Key_W:
            self.keys.forward = True
        elif k == QtCore.Qt.Key_S:
            self.keys.back = True
        elif k == QtCore.Qt.Key_A:
            self.keys.left = True
        elif k == QtCore.Qt.Key_D:
            self.keys.right = True
        elif k == QtCore.Qt.Key_Q:
            self.keys.yaw_left = True
        elif k == QtCore.Qt.Key_E:
            self.keys.yaw_right = True
        elif k == QtCore.Qt.Key_Space:
            self._stop()

    def keyReleaseEvent(self, ev: QtGui.QKeyEvent):
        if ev.isAutoRepeat():
            return
        k = ev.key()
        if k == QtCore.Qt.Key_W:
            self.keys.forward = False
        elif k == QtCore.Qt.Key_S:
            self.keys.back = False
        elif k == QtCore.Qt.Key_A:
            self.keys.left = False
        elif k == QtCore.Qt.Key_D:
            self.keys.right = False
        elif k == QtCore.Qt.Key_Q:
            self.keys.yaw_left = False
        elif k == QtCore.Qt.Key_E:
            self.keys.yaw_right = False

    # ---- ROS / UI loops ----
    def _spin_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            pass

    def _refresh_ui(self):
        # battery
        b = self.node.last_battery
        if b is not None:
            pct = b.percentage if (b.percentage is not None and b.percentage >= 0) else float("nan")
            v = b.voltage
            self.lbl_batt.setText(f"Battery: {pct*100:.0f}%  {v:.2f}V  {b.current:.2f}A" if pct == pct else f"Battery: --  {v:.2f}V")
        else:
            self.lbl_batt.setText("Battery: --")

        # odom
        o = self.node.last_odom
        if o is not None:
            x = o.pose.pose.position.x
            y = o.pose.pose.position.y
            yaw = yaw_from_quat(o.pose.pose.orientation)
            self.lbl_odom.setText(f"Odom: x={x:+.2f} y={y:+.2f} yaw={yaw:+.2f} rad")
        else:
            self.lbl_odom.setText("Odom: --")

        # imu
        imu = self.node.last_imu
        if imu is not None:
            yaw = yaw_from_quat(imu.orientation)
            self.lbl_imu.setText(f"IMU: yaw={yaw:+.2f} rad  wz={imu.angular_velocity.z:+.2f} rad/s")
        else:
            self.lbl_imu.setText("IMU: --")

        # diagnostics summary
        d = self.node.last_diag
        if d is not None and d.status:
            worst = max(s.level for s in d.status)
            self.lbl_diag.setText(f"Diagnostics: {len(d.status)} entries  worst_level={worst}")
        else:
            self.lbl_diag.setText("Diagnostics: --")

        # force sensors
        f = getattr(self.node, 'last_force', None)
        if f is not None and len(f) == 6:
            self.lbl_force.setText('Force[mV]: ' + ' '.join([f'{v:4d}' for v in f]))
            self.force_plot.append(time.time(), [float(v) for v in f])
        else:
            self.lbl_force.setText('Force: --')

        # imu plot (use /imu/data_raw by default)
        imu = self.node.last_imu
        if imu is not None:
            vals = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
            self.imu_plot.append(time.time(), vals)

        # cameras
        if self.node.last_color_qimage is not None:
            meta = f"{self.node._color_fps:.1f} fps  {self.node.color_topic}"
            self.color_w.set_image(self.node.last_color_qimage, meta)
        else:
            self.color_w.set_image(None, f"{self.node.color_topic}")

        if self.node.last_depth_qimage is not None:
            c = self.node._depth_center_m
            m = self.node._depth_min_m
            meta = f"{self.node._depth_fps:.1f} fps  center={c:.2f}m min={m:.2f}m" if (c and m) else f"{self.node.depth_topic}"
            self.depth_w.set_image(self.node.last_depth_qimage, meta)
        else:
            self.depth_w.set_image(None, f"{self.node.depth_topic}")

        # refresh ROS graph every 2s
        now = time.time()
        if now - self._last_graph_refresh > 2.0:
            self._last_graph_refresh = now
            try:
                graph = dict(self.node.get_topic_names_and_types())
                for i, t in enumerate(IMPORTANT_TOPICS):
                    present = "yes" if t in graph else "no"
                    types = ", ".join(graph.get(t, []))
                    self.topic_table.item(i, 1).setText(present)
                    self.topic_table.item(i, 2).setText(types)
            except Exception:
                pass

    # ---- publishers bound to UI ----
    def _stop(self):
        self.keys = KeyState()
        if self.teleop_enabled:
            self.node.publish_cmd_vel(0.0, 0.0, 0.0)
        self.lbl_cmd.setText("cmd: vx=0 vy=0 wz=0")

    def _publish_cmd_from_keys(self):
        if not self.teleop_enabled:
            return
        vx = 0.0
        vy = 0.0
        wz = 0.0
        if self.keys.forward: vx += self.vx_scale
        if self.keys.back: vx -= self.vx_scale
        if self.keys.left: vy += self.vy_scale
        if self.keys.right: vy -= self.vy_scale
        if self.keys.yaw_left: wz += self.wz_scale
        if self.keys.yaw_right: wz -= self.wz_scale

        self.node.publish_cmd_vel(vx, vy, wz)
        self.lbl_cmd.setText(f"cmd: vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}")

    def _publish_mode(self):
        mode = self.combo_mode.currentText()
        self.node.publish_mode(mode)

    def _publish_rider(self, state):
        present = state == QtCore.Qt.Checked
        self.node.publish_rider(present)

    def _toggle_estop(self):
        current = bool(self.node.last_estop) if self.node.last_estop is not None else False
        self.node.publish_estop(not current)

    def _publish_gait_mode(self):
        v = self.gait_edit.text().strip()
        if v:
            self.node.publish_gait_mode(v)


def main():
    rclpy.init()
    node = GuiNode()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node)
    win.show()

    ret = app.exec_()

    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()