"""
FILE LOCATION: fuad_ws/src/integration_pkg/integration_pkg/teleop_bridge.py

Teleop Bridge (Manual Controller)
----------------------------------
Reads keyboard input and publishes Twist messages to /manual_cmd.

USAGE:
  # Interactive terminal:
  ros2 run integration_pkg teleop_bridge

  # From launch file (non-TTY) — falls back to subscriber mode:
  ros2 launch integration_pkg all.launch.py

KEYS (interactive mode):
  w / s  : forward / backward (linear.x)
  a / d  : left / right turn (angular.z)
  q / e  : up / down (linear.z)
  space  : stop
  Ctrl+C : quit

NOTES:
  This node does NOT publish directly to /drone/cmd_vel. It publishes to
  /manual_cmd so the selector_node can arbitrate between manual and voice.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

# termios/tty only available on Unix-like systems with a real TTY
try:
    import termios
    import tty
    import select
    HAS_TTY = True
except ImportError:
    HAS_TTY = False


class TeleopBridge(Node):
    def __init__(self):
        super().__init__("teleop_bridge")

        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        manual_topic = self.get_parameter("manual_cmd_topic").value

        self.pub = self.create_publisher(Twist, manual_topic, 10)
        self.twist = Twist()

        # Check if we have a real terminal
        self.has_tty = HAS_TTY and sys.stdin.isatty()

        if self.has_tty:
            self.get_logger().info(
                "TeleopBridge started in INTERACTIVE mode.\n"
                "Keys: w/s=forward/back, a/d=turn, q/e=up/down, space=stop"
            )
        else:
            self.get_logger().warn(
                """
                TeleopBridge: no TTY detected (running from launch file?).
                Interactive keyboard control disabled.
                Use: ros2 topic pub /manual_cmd geometry_msgs/Twist
                """
            )
            # In non-TTY mode, we still spin so the node stays alive.
            # The user can publish to /manual_cmd via CLI or another node.

    def get_key(self):
        if not self.has_tty:
            return None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        if not self.has_tty:
            # Non-interactive mode: just spin and let external publishers
            # send commands to /manual_cmd
            rclpy.spin(self)
            return

        self.settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key is None:
                    continue
                if key == "\x03":  # Ctrl+C
                    break
                elif key == "w":
                    self.twist.linear.x = 1.0
                elif key == "s":
                    self.twist.linear.x = -1.0
                elif key == "a":
                    self.twist.angular.z = 1.0
                elif key == "d":
                    self.twist.angular.z = -1.0
                elif key == "q":
                    self.twist.linear.z = 1.0
                elif key == "e":
                    self.twist.linear.z = -1.0
                elif key == " ":
                    self.twist = Twist()
                else:
                    # Key released — stop (simple oneshot behavior)
                    self.twist = Twist()

                self.pub.publish(self.twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
