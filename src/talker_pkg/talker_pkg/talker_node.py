#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker_node')

        self.declare_parameter('message_prefix', 'Hello ROS2')
        self.declare_parameter('publish_period_sec', 1.0)

        self.enabled = True
        self.counter = 0

        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.toggle_subscriber_ = self.create_subscription(
            Bool,
            'talker_enabled',
            self.toggle_callback,
            10,
        )
        publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self.timer = self.create_timer(publish_period_sec, self.timer_callback)

        self.get_logger().info("Talker node started.")
        self.get_logger().info(
            f"Parameters: message_prefix="
            f"'{self.get_parameter('message_prefix').value}', "
            f"publish_period_sec={publish_period_sec}"
        )

    def timer_callback(self):
        if not self.enabled:
            return

        msg = String()
        prefix = str(self.get_parameter('message_prefix').value)
        msg.data = f"{prefix} {self.counter}"
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1

    def toggle_callback(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info(f"Talker enabled: {self.enabled}")


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
