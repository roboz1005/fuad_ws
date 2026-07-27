"""
Simulated battery publisher for the drone dashboard.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatterySimNode(Node):
    def __init__(self):
        super().__init__("battery_sim")
        self.declare_parameter("battery_topic", "/drone/battery")
        self.declare_parameter("initial_charge", 100.0)
        self.declare_parameter("drain_rate_per_sec", 0.05)

        topic = self.get_parameter("battery_topic").value
        self.charge = self.get_parameter("initial_charge").value
        self.drain = self.get_parameter("drain_rate_per_sec").value

        self.pub = self.create_publisher(BatteryState, topic, 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(f"BatterySim publishing on {topic}")

    def _publish(self):
        self.charge = max(0.0, self.charge - self.drain)
        msg = BatteryState()
        msg.percentage = self.charge / 100.0
        msg.voltage = 12.0 * (self.charge / 100.0)
        msg.present = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
