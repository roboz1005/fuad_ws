"""
TF2-based TurtleBot Follower (Bonus — 15 points)
Uses TF transforms instead of raw odometry for smoother tracking.
"""
import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFFollowNode(Node):
    def __init__(self):
        super().__init__("tf_follow_controller")

        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("max_linear", 0.22)
        self.declare_parameter("max_angular", 2.84)
        self.declare_parameter("follow_distance", 1.5)
        self.declare_parameter("avoidance_distance", 0.35)
        self.declare_parameter("tb3_cmd_vel_topic", "/model/turtlebot3/cmd_vel")
        self.declare_parameter("tb3_scan_topic", "/model/turtlebot3/scan")
        self.declare_parameter("leader_frame", "X3/base_link")
        self.declare_parameter("follower_frame", "turtlebot3/base_link")

        self.kp_linear = self.get_parameter("kp_linear").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.avoidance_distance = self.get_parameter("avoidance_distance").value
        self.leader_frame = self.get_parameter("leader_frame").value
        self.follower_frame = self.get_parameter("follower_frame").value

        tb3_cmd_topic = self.get_parameter("tb3_cmd_vel_topic").value
        tb3_scan_topic = self.get_parameter("tb3_scan_topic").value

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan = None
        self.create_subscription(LaserScan, tb3_scan_topic, self._scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, tb3_cmd_topic, 10)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"TFFollowNode: {self.follower_frame} → {self.leader_frame}"
        )

    def _scan_cb(self, msg):
        self.scan = msg

    def _control_loop(self):
        # Lookup transform: follower → leader
        try:
            trans = self.tf_buffer.lookup_transform(
                self.follower_frame,
                self.leader_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed: {e}", throttle_duration_sec=2.0
            )
            return

        # Obstacle avoidance first
        avoid_twist = self._obstacle_avoidance()
        if avoid_twist is not None:
            self.cmd_pub.publish(avoid_twist)
            return

        # Relative position in follower frame
        dx = trans.transform.translation.x
        dy = trans.transform.translation.y
        distance = math.hypot(dx, dy)

        if distance < self.follow_distance:
            self.cmd_pub.publish(Twist())
            return

        # Heading error = angle to leader in follower frame
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw)

        twist = Twist()
        twist.linear.x = min(
            self.kp_linear * (distance - self.follow_distance),
            self.max_linear
        )
        twist.angular.z = max(
            min(self.kp_angular * yaw_error, self.max_angular),
            -self.max_angular,
        )
        self.cmd_pub.publish(twist)

    def _obstacle_avoidance(self):
        if self.scan is None:
            return None
        ranges = self.scan.ranges
        if not ranges:
            return None

        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        threshold = self.avoidance_distance

        front_min = threshold
        left_min = threshold
        right_min = threshold

        for i, dist in enumerate(ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue
            angle = angle_min + i * angle_inc
            if abs(angle) <= math.radians(30):
                front_min = min(front_min, dist)
            elif 0 < angle <= math.radians(90):
                left_min = min(left_min, dist)
            elif -math.radians(90) <= angle < 0:
                right_min = min(right_min, dist)

        twist = Twist()
        if front_min < threshold:
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular if left_min > right_min else -self.max_angular
            return twist
        if left_min < threshold:
            twist.angular.z = -self.max_angular * 0.5
            return twist
        if right_min < threshold:
            twist.angular.z = self.max_angular * 0.5
            return twist
        return None

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TFFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
