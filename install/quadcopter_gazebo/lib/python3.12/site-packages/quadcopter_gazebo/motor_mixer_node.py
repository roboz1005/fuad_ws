"""
Motor Mixer Node (X3 UAV)
-------------------------
Subscribes to /drone/cmd_vel (geometry_msgs/Twist).
Converts to 4 rotor throttle commands and publishes motor speeds.

Two modes:
  1. Bridge mode (default): Publishes actuator_msgs/Actuators to a ROS 2 topic
     that gets bridged to Gazebo via ros_gz_bridge.
  2. Subprocess mode (fallback): Uses `gz topic` CLI directly.

DEPENDENCIES: actuator_msgs (install: sudo apt install ros-jazzy-actuator-msgs)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from actuator_msgs.msg import Actuators
import subprocess
import tempfile
import os


class MotorMixerNode(Node):
    def __init__(self):
        super().__init__("motor_mixer")

        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("gz_topic", "/model/X3/command/motor_speed")
        self.declare_parameter("hover_throttle", 0.55)
        self.declare_parameter("max_tilt", 0.3)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("max_vz", 1.0)
        self.declare_parameter("gain", 0.15)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("publish_via_bridge", True)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.gz_topic = self.get_parameter("gz_topic").value
        self.hover = self.get_parameter("hover_throttle").value
        self.max_tilt = self.get_parameter("max_tilt").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.max_vz = self.get_parameter("max_vz").value
        self.gain = self.get_parameter("gain").value
        rate = self.get_parameter("publish_rate").value
        self.use_bridge = self.get_parameter("publish_via_bridge").value

        self.cmd = Twist()
        self.create_subscription(Twist, cmd_topic, self.twist_cb, 10)
        self.create_timer(1.0 / rate, self.publish_loop)

        if self.use_bridge:
            self.motor_pub = self.create_publisher(Actuators, self.gz_topic, 10)
            self.get_logger().info(
                f"MotorMixer (BRIDGE MODE) started.\n"
                f"  Subscribing to: {cmd_topic}\n"
                f"  Publishing Actuators to ROS topic: {self.gz_topic}\n"
                f"  Rate: {rate} Hz"
            )
        else:
            self.motor_pub = None
            self.get_logger().info(
                f"MotorMixer (SUBPROCESS MODE) started.\n"
                f"  Subscribing to: {cmd_topic}\n"
                f"  Publishing via gz CLI to: {self.gz_topic}\n"
                f"  Rate: {rate} Hz"
            )

        self._first_cmd = True

    def twist_cb(self, msg: Twist):
        if self._first_cmd:
            self.get_logger().info(
                f"First cmd_vel received: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, "
                f"{msg.linear.z:.2f}) angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, "
                f"{msg.angular.z:.2f})"
            )
            self._first_cmd = False
        self.cmd = msg

    def publish_loop(self):
        # ---- Desired setpoints (clamped) ----
        pitch = max(-self.max_tilt, min(self.max_tilt, -self.cmd.linear.x))
        roll = max(-self.max_tilt, min(self.max_tilt, -self.cmd.linear.y))
        yaw = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.cmd.angular.z))
        vz = max(-self.max_vz, min(self.max_vz, self.cmd.linear.z))

        throttle = self.hover + vz * 0.1

        # ---- Mixer ----
        m0 = throttle + (-pitch - roll - yaw) * self.gain
        m1 = throttle + (+pitch - roll - yaw) * self.gain
        m2 = throttle + (+pitch + roll + yaw) * self.gain
        m3 = throttle + (-pitch + roll + yaw) * self.gain

        m0 = max(0.0, min(1.0, m0))
        m1 = max(0.0, min(1.0, m1))
        m2 = max(0.0, min(1.0, m2))
        m3 = max(0.0, min(1.0, m3))

        if self.use_bridge and self.motor_pub is not None:
            # FIX: Use msg.normalized (not velocity) — MulticopterMotorModel reads normalized field
            msg = Actuators()
            msg.normalized = [m0, m1, m2, m3]
            self.motor_pub.publish(msg)
        else:
            # Fallback: subprocess gz topic
            proto_text = (
                f"normalized: {m0:.6f}\n"
                f"normalized: {m1:.6f}\n"
                f"normalized: {m2:.6f}\n"
                f"normalized: {m3:.6f}\n"
            )
            tmp_path = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode='w', suffix='.txt', delete=False
                ) as tmp:
                    tmp.write(proto_text)
                    tmp_path = tmp.name

                result = subprocess.run(
                    [
                        "gz", "topic", "-t", self.gz_topic,
                        "-m", "gz.msgs.Actuators",
                        "-p", tmp_path,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                )
                if result.returncode != 0 and result.stderr:
                    self.get_logger().warn(
                        f"gz topic error: {result.stderr.strip()}",
                        throttle_duration_sec=5.0,
                    )
            except Exception as e:
                self.get_logger().warn(
                    f"Motor publish failed: {e}",
                    throttle_duration_sec=5.0,
                )
            finally:
                if tmp_path and os.path.exists(tmp_path):
                    os.remove(tmp_path)


def main(args=None):
    rclpy.init(args=args)
    node = MotorMixerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()