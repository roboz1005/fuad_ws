"""
Command Selector Node (Dual-Mode Switching)
--------------------------------------------
Arbitrates manual vs voice Twist commands and publishes /drone/cmd_vel.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class SelectorNode(Node):
    def __init__(self):
        super().__init__("command_selector")

        self.declare_parameter("drone_cmd_vel_topic", "/drone/cmd_vel")
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("voice_cmd_topic", "/voice_cmd")
        self.declare_parameter("mode_topic", "/control/mode")
        self.declare_parameter("set_mode_service", "/command_selector/set_mode")

        drone_cmd_topic = self.get_parameter("drone_cmd_vel_topic").value
        manual_topic = self.get_parameter("manual_cmd_topic").value
        voice_topic = self.get_parameter("voice_cmd_topic").value
        mode_topic = self.get_parameter("mode_topic").value
        set_mode_service = self.get_parameter("set_mode_service").value

        self.voice_mode = False
        self.manual_twist = Twist()
        self.voice_twist = Twist()

        self.create_subscription(Twist, manual_topic, self._manual_cb, 10)
        self.create_subscription(Twist, voice_topic, self._voice_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, drone_cmd_topic, 10)
        self.mode_pub = self.create_publisher(Bool, mode_topic, 10)
        self.create_service(SetBool, set_mode_service, self._set_mode_cb)
        self.create_timer(0.05, self._publish_loop)

        self._publish_mode()
        self.get_logger().info(
            "SelectorNode started in MANUAL mode. "
            f"Service: {set_mode_service}"
        )

    def _manual_cb(self, msg: Twist):
        if not self.voice_mode:
            self.manual_twist = msg

    def _voice_cb(self, msg: Twist):
        if self.voice_mode:
            self.voice_twist = msg

    def _set_mode_cb(self, request, response):
        self.voice_mode = request.data
        mode_str = "VOICE" if self.voice_mode else "MANUAL"
        self.get_logger().info(f"Mode switched to: {mode_str}")
        self._publish_mode()
        response.success = True
        response.message = f"Mode set to {mode_str}"
        return response

    def _publish_mode(self):
        msg = Bool()
        msg.data = self.voice_mode
        self.mode_pub.publish(msg)

    def _publish_loop(self):
        if self.voice_mode:
            self.cmd_pub.publish(self.voice_twist)
        else:
            self.cmd_pub.publish(self.manual_twist)


def main(args=None):
    rclpy.init(args=args)
    node = SelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
