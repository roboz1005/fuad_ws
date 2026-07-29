"""
Gazebo TF Broadcaster
---------------------
Subscribes to odometry topics (already bridged from Gazebo) and broadcasts
poses as TF2 transforms so TF follow nodes can lookup transforms.

Supports multiple robots via parameter arrays.

Publishes:
  /tf  (world → X3/base_link, world → turtlebot3/base_link, etc.)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__("gazebo_tf_broadcaster")

        # Default: drone + one turtlebot3
        self.declare_parameter("odom_topics", ["/model/X3/odometry", "/model/turtlebot3/odometry"])
        self.declare_parameter("frame_ids", ["X3/base_link", "turtlebot3/base_link"])
        self.declare_parameter("world_frame", "world")

        odom_topics = self.get_parameter("odom_topics").value
        frame_ids = self.get_parameter("frame_ids").value
        self.world_frame = self.get_parameter("world_frame").value

        if len(odom_topics) != len(frame_ids):
            self.get_logger().error("odom_topics and frame_ids must have same length!")
            return

        self.tf_broadcaster = TransformBroadcaster(self)

        for topic, frame in zip(odom_topics, frame_ids):
            self.create_subscription(
                Odometry, topic,
                lambda msg, f=frame: self._broadcast(msg, f),
                10
            )
            self.get_logger().info(f"Subscribing to {topic} → TF frame {frame}")

        self.get_logger().info(
            f"TF Broadcaster ready: {self.world_frame} → {frame_ids}"
        )

    def _broadcast(self, msg: Odometry, child_frame: str):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
