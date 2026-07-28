"""
Motor Mixer Node (X3 UAV)
-------------------------
Subscribes to /drone/cmd_vel (geometry_msgs/Twist).
Converts to 4 rotor throttle commands and publishes actuator_msgs/Actuators.

The ros_gz_bridge forwards /model/X3/command/motor_speed to Gazebo.
The MulticopterMotorModel plugin listens on /model/X3/command/motor_speed.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from actuator_msgs.msg import Actuators


class MotorMixerNode(Node):
    def __init__(self):
        super().__init__("motor_mixer")

        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("motor_topic", "/model/X3/command/motor_speed")
        self.declare_parameter("hover_throttle", 0.55)
        self.declare_parameter("max_tilt", 0.3)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("max_vz", 1.0)
        self.declare_parameter("gain", 0.15)
        self.declare_parameter("publish_rate", 20.0)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        motor_topic = self.get_parameter("motor_topic").value
        self.hover = self.get_parameter("hover_throttle").value
        self.max_tilt = self.get_parameter("max_tilt").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.max_vz = self.get_parameter("max_vz").value
        self.gain = self.get_parameter("gain").value
        rate = self.get_parameter("publish_rate").value

        self.cmd = Twist()
        self.create_subscription(Twist, cmd_topic, self.twist_cb, 10)
        self.create_timer(1.0 / rate, self.publish_loop)

        self.motor_pub = self.create_publisher(Actuators, motor_topic, 10)

        self.get_logger().info(
            f"MotorMixer started.\n"
            f"  Subscribing to: {cmd_topic}\n"
            f"  Publishing Actuators to: {motor_topic}\n"
            f"  Rate: {rate} Hz"
        )

        self._first_cmd = True
        self._pub_count = 0

    def twist_cb(self, msg: Twist):
        if self._first_cmd:
            self.get_logger().info(
                f"First cmd_vel: linear=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
                f"angular=({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})"
            )
            self._first_cmd = False
        self.cmd = msg

    def publish_loop(self):
        pitch = max(-self.max_tilt, min(self.max_tilt, -self.cmd.linear.x))
        roll  = max(-self.max_tilt, min(self.max_tilt, -self.cmd.linear.y))
        yaw   = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.cmd.angular.z))
        vz    = max(-self.max_vz, min(self.max_vz, self.cmd.linear.z))

        throttle = self.hover + vz * 0.1

        m0 = throttle + (-pitch - roll - yaw) * self.gain
        m1 = throttle + (+pitch - roll - yaw) * self.gain
        m2 = throttle + (+pitch + roll + yaw) * self.gain
        m3 = throttle + (-pitch + roll + yaw) * self.gain

        m0 = max(0.0, min(1.0, m0))
        m1 = max(0.0, min(1.0, m1))
        m2 = max(0.0, min(1.0, m2))
        m3 = max(0.0, min(1.0, m3))

        msg = Actuators()
        msg.normalized = [m0, m1, m2, m3]
        self.motor_pub.publish(msg)

        self._pub_count += 1
        if self._pub_count % 100 == 0:
            self.get_logger().info(
                f"Motors: [{m0:.3f}, {m1:.3f}, {m2:.3f}, {m3:.3f}]"
            )


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