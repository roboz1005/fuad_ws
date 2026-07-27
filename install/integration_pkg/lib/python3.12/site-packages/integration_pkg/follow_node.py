"""
TurtleBot3 Follow Controller with Obstacle Avoidance
-----------------------------------------------------
Leader-follower using odometry with reactive LaserScan avoidance.
Priority: obstacle avoidance > leader following.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FollowNode(Node):
    def __init__(self):
        super().__init__("follow_controller")

        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("max_linear", 0.22)
        self.declare_parameter("max_angular", 2.84)
        self.declare_parameter("follow_distance", 1.5)
        self.declare_parameter("avoidance_distance", 0.35)
        self.declare_parameter("drone_odom_topic", "/drone/odom")
        self.declare_parameter("tb3_odom_topic", "/turtlebot/odom")
        self.declare_parameter("tb3_cmd_vel_topic", "/turtlebot/cmd_vel")
        self.declare_parameter("tb3_scan_topic", "/turtlebot/scan")

        self.kp_linear = self.get_parameter("kp_linear").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.avoidance_distance = self.get_parameter("avoidance_distance").value

        drone_odom_topic = self.get_parameter("drone_odom_topic").value
        tb3_odom_topic = self.get_parameter("tb3_odom_topic").value
        tb3_cmd_topic = self.get_parameter("tb3_cmd_vel_topic").value
        tb3_scan_topic = self.get_parameter("tb3_scan_topic").value

        self.drone_odom = None
        self.tb3_odom = None
        self.scan = None

        self.create_subscription(Odometry, drone_odom_topic, self._drone_odom_cb, 10)
        self.create_subscription(Odometry, tb3_odom_topic, self._tb3_odom_cb, 10)
        self.create_subscription(LaserScan, tb3_scan_topic, self._scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, tb3_cmd_topic, 10)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info("FollowNode started. Waiting for odometry and scan...")

    def _drone_odom_cb(self, msg):
        self.drone_odom = msg

    def _tb3_odom_cb(self, msg):
        self.tb3_odom = msg

    def _scan_cb(self, msg):
        self.scan = msg

    def _control_loop(self):
        if self.drone_odom is None or self.tb3_odom is None:
            return

        avoid_twist = self._obstacle_avoidance()
        if avoid_twist is not None:
            self.cmd_pub.publish(avoid_twist)
            return

        drone_x = self.drone_odom.pose.pose.position.x
        drone_y = self.drone_odom.pose.pose.position.y
        tb_x = self.tb3_odom.pose.pose.position.x
        tb_y = self.tb3_odom.pose.pose.position.y
        tb_yaw = self._quat_to_yaw(self.tb3_odom.pose.pose.orientation)

        dx = drone_x - tb_x
        dy = drone_y - tb_y
        distance = math.hypot(dx, dy)

        if distance < self.follow_distance:
            self.cmd_pub.publish(Twist())
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw - tb_yaw)

        twist = Twist()
        twist.linear.x = min(self.kp_linear * (distance - self.follow_distance), self.max_linear)
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
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
