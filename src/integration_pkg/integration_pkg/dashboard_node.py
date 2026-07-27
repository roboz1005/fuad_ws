"""
Telemetry Dashboard (PyQt5)
---------------------------
Real-time GUI with dual-mode control, telemetry, and manual command buttons.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


def _quat_to_euler(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class RosSignalBridge(QObject):
    """Qt signal bridge — must inherit QObject, not rclpy Node."""

    sig_odom = pyqtSignal(object)
    sig_battery = pyqtSignal(object)
    sig_image = pyqtSignal(object)
    sig_voice = pyqtSignal(str)
    sig_mode = pyqtSignal(bool)


class DashboardNode(Node):
    def __init__(self, signals: RosSignalBridge):
        super().__init__("dashboard")
        self.signals = signals
        self.voice_mode = False

        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("voice_cmd_topic", "/voice_cmd")
        self.declare_parameter("drone_odom_topic", "/drone/odom")
        self.declare_parameter("drone_battery_topic", "/drone/battery")
        self.declare_parameter("drone_camera_topic", "/drone/camera")
        self.declare_parameter("mode_topic", "/control/mode")
        self.declare_parameter("set_mode_service", "/command_selector/set_mode")
        manual_topic = self.get_parameter("manual_cmd_topic").value
        voice_topic = self.get_parameter("voice_cmd_topic").value
        odom_topic = self.get_parameter("drone_odom_topic").value
        battery_topic = self.get_parameter("drone_battery_topic").value
        camera_topic = self.get_parameter("drone_camera_topic").value
        mode_topic = self.get_parameter("mode_topic").value
        set_mode_service = self.get_parameter("set_mode_service").value

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(BatteryState, battery_topic, self._battery_cb, 10)
        self.create_subscription(Image, camera_topic, self._camera_cb, 10)
        self.create_subscription(Twist, voice_topic, self._voice_cb, 10)
        self.create_subscription(Bool, mode_topic, self._mode_cb, 10)

        self.manual_pub = self.create_publisher(Twist, manual_topic, 10)
        self.mode_client = self.create_client(SetBool, set_mode_service)

    def _odom_cb(self, msg):
        self.signals.sig_odom.emit(msg)

    def _battery_cb(self, msg):
        self.signals.sig_battery.emit(msg)

    def _camera_cb(self, msg):
        self.signals.sig_image.emit(msg)

    def _voice_cb(self, msg):
        parts = []
        if msg.linear.x > 0.1:
            parts.append("Forward")
        elif msg.linear.x < -0.1:
            parts.append("Backward")
        if msg.linear.z > 0.1:
            parts.append("Up")
        elif msg.linear.z < -0.1:
            parts.append("Down")
        if msg.angular.z > 0.1:
            parts.append("Rotate Left")
        elif msg.angular.z < -0.1:
            parts.append("Rotate Right")
        if not parts:
            parts.append("Stop")
        self.signals.sig_voice.emit("Voice: " + ", ".join(parts))

    def _mode_cb(self, msg):
        self.voice_mode = msg.data
        self.signals.sig_mode.emit(msg.data)

    def publish_manual(self, twist: Twist):
        if self.voice_mode:
            self.get_logger().warn("Manual command ignored — VOICE mode active")
            return
        self.manual_pub.publish(twist)

    def set_mode(self, voice_mode: bool):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/command_selector/set_mode not available")
            return
        req = SetBool.Request()
        req.data = voice_mode
        self.mode_client.call_async(req)


class DashboardWindow(QWidget):
    CMD_LINEAR = 1.0
    CMD_ANGULAR = 1.0
    CMD_VERTICAL = 1.0

    def __init__(self, ros_node: DashboardNode, signals: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Drone Telemetry Dashboard")
        self.setMinimumSize(900, 700)

        root = QVBoxLayout()

        # Mode row
        mode_row = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL MODE")
        self.btn_voice = QPushButton("VOICE MODE")
        self.btn_manual.setCheckable(True)
        self.btn_voice.setCheckable(True)
        self.btn_manual.setChecked(True)
        self.btn_manual.clicked.connect(lambda: self._switch_mode(False))
        self.btn_voice.clicked.connect(lambda: self._switch_mode(True))
        mode_row.addWidget(self.btn_manual)
        mode_row.addWidget(self.btn_voice)
        self.lbl_mode = QLabel("Mode: MANUAL")
        mode_row.addWidget(self.lbl_mode)
        root.addLayout(mode_row)

        # Telemetry
        telem = QGroupBox("Telemetry")
        grid = QGridLayout()
        self.lbl_pos = QLabel("Position: X=-- Y=-- Z=--")
        self.lbl_orient = QLabel("Roll/Pitch/Yaw: -- / -- / --")
        self.lbl_vel = QLabel("Linear vel: -- | Angular vel: --")
        self.lbl_battery = QLabel("Battery: -- %")
        self.lbl_voice = QLabel("Voice command: --")
        self.lbl_status = QLabel("Flight status: HOVER")
        self.lbl_time = QLabel("Timestamp: --")
        grid.addWidget(self.lbl_pos, 0, 0)
        grid.addWidget(self.lbl_orient, 0, 1)
        grid.addWidget(self.lbl_vel, 1, 0)
        grid.addWidget(self.lbl_battery, 1, 1)
        grid.addWidget(self.lbl_voice, 2, 0)
        grid.addWidget(self.lbl_status, 2, 1)
        grid.addWidget(self.lbl_time, 3, 0, 1, 2)
        telem.setLayout(grid)
        root.addWidget(telem)

        # Manual controls
        ctrl = QGroupBox("Manual Control (disabled in Voice Mode)")
        ctrl_layout = QGridLayout()
        self.manual_buttons = {}
        commands = [
            ("Takeoff", self._cmd_takeoff, 0, 1),
            ("Forward", lambda: self._send_cmd(lx=self.CMD_LINEAR), 0, 2),
            ("Land", self._cmd_land, 0, 3),
            ("Left", lambda: self._send_cmd(az=self.CMD_ANGULAR), 1, 0),
            ("Hover", self._cmd_stop, 1, 1),
            ("Right", lambda: self._send_cmd(az=-self.CMD_ANGULAR), 1, 2),
            ("Up", lambda: self._send_cmd(lz=self.CMD_VERTICAL), 2, 1),
            ("Backward", lambda: self._send_cmd(lx=-self.CMD_LINEAR), 3, 1),
            ("Down", lambda: self._send_cmd(lz=-self.CMD_VERTICAL), 4, 1),
            ("Rotate L", lambda: self._send_cmd(az=self.CMD_ANGULAR), 2, 0),
            ("Rotate R", lambda: self._send_cmd(az=-self.CMD_ANGULAR), 2, 2),
        ]
        for label, handler, row, col in commands:
            btn = QPushButton(label)
            btn.clicked.connect(handler)
            ctrl_layout.addWidget(btn, row, col)
            self.manual_buttons[label] = btn

        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setStyleSheet(
            "background-color: #cc0000; color: white; font-weight: bold; min-height: 48px;"
        )
        self.btn_stop.clicked.connect(self._cmd_stop)
        ctrl_layout.addWidget(self.btn_stop, 5, 0, 1, 3)
        ctrl.setLayout(ctrl_layout)
        root.addWidget(ctrl)

        # Camera + log
        self.lbl_camera = QLabel("Camera Feed")
        self.lbl_camera.setMinimumSize(480, 270)
        self.lbl_camera.setStyleSheet("background-color: black; color: #888;")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        root.addWidget(self.lbl_camera)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        root.addWidget(self.log)

        self.setLayout(root)

        signals.sig_odom.connect(self._update_odom)
        signals.sig_battery.connect(self._update_battery)
        signals.sig_image.connect(self._update_image)
        signals.sig_voice.connect(self._update_voice)
        signals.sig_mode.connect(self._update_mode_ui)

        self._manual_timer = QTimer()
        self._manual_timer.timeout.connect(self._republish_active_cmd)
        self._active_twist = Twist()
        self._manual_timer.start(50)

    def _switch_mode(self, voice: bool):
        self.ros_node.set_mode(voice)
        self.log.append(f"Requesting {'VOICE' if voice else 'MANUAL'} mode")

    def _update_mode_ui(self, voice: bool):
        self.btn_manual.setChecked(not voice)
        self.btn_voice.setChecked(voice)
        self.lbl_mode.setText(f"Mode: {'VOICE' if voice else 'MANUAL'}")
        for btn in self.manual_buttons.values():
            btn.setEnabled(not voice)
        self.btn_stop.setEnabled(True)

    def _send_cmd(self, lx=0.0, lz=0.0, az=0.0):
        twist = Twist()
        twist.linear.x = lx
        twist.linear.z = lz
        twist.angular.z = az
        self._active_twist = twist
        self.ros_node.publish_manual(twist)
        self.lbl_status.setText("Flight status: MOVING")

    def _cmd_stop(self):
        self._active_twist = Twist()
        self.ros_node.publish_manual(Twist())
        self.lbl_status.setText("Flight status: HOVER")

    def _cmd_takeoff(self):
        self._send_cmd(lz=self.CMD_VERTICAL)

    def _cmd_land(self):
        self._send_cmd(lz=-self.CMD_VERTICAL)
        self.lbl_status.setText("Flight status: LANDING")

    def _republish_active_cmd(self):
        if self.ros_node.voice_mode:
            return
        if (
            self._active_twist.linear.x != 0.0
            or self._active_twist.linear.z != 0.0
            or self._active_twist.angular.z != 0.0
        ):
            self.ros_node.publish_manual(self._active_twist)

    def _update_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_euler(q)
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        speed = math.sqrt(lv.x ** 2 + lv.y ** 2 + lv.z ** 2)
        ang = math.sqrt(av.x ** 2 + av.y ** 2 + av.z ** 2)

        self.lbl_pos.setText(f"Position: X={p.x:.2f} Y={p.y:.2f} Z={p.z:.2f}")
        self.lbl_orient.setText(
            f"Roll/Pitch/Yaw: {math.degrees(roll):.1f}° / "
            f"{math.degrees(pitch):.1f}° / {math.degrees(yaw):.1f}°"
        )
        self.lbl_vel.setText(f"Linear vel: {speed:.2f} m/s | Angular vel: {ang:.2f} rad/s")
        stamp = msg.header.stamp
        self.lbl_time.setText(f"Timestamp: {stamp.sec}.{stamp.nanosec:09d}")

    def _update_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0 if msg.percentage >= 0.0 else msg.voltage
        self.lbl_battery.setText(f"Battery: {pct:.0f} %")

    def _update_image(self, msg: Image):
        data = bytes(msg.data)
        if msg.encoding == "rgb8":
            qimg = QImage(data, msg.width, msg.height, msg.step, QImage.Format_RGB888)
        elif msg.encoding == "bgr8":
            qimg = QImage(data, msg.width, msg.height, msg.step, QImage.Format_BGR888)
        elif msg.encoding in ("rgba8", "bgra8"):
            qimg = QImage(data, msg.width, msg.height, msg.step, QImage.Format_RGBA8888)
        else:
            return
        qimg = qimg.copy()
        pixmap = QPixmap.fromImage(qimg)
        self.lbl_camera.setPixmap(
            pixmap.scaled(self.lbl_camera.width(), self.lbl_camera.height(), Qt.KeepAspectRatio)
        )

    def _update_voice(self, text: str):
        self.lbl_voice.setText(text)


def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)

    signals = RosSignalBridge()
    ros_node = DashboardNode(signals)
    window = DashboardWindow(ros_node, signals)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    spin_timer.start(10)

    exit_code = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
