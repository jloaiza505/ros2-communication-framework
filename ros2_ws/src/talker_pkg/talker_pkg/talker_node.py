import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker_node')

        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.enabled = True
        self.counter = 0

        self.get_logger().info("Talker node started.")

    def timer_callback(self):
        if not self.enabled:
            return

        msg = String()
        msg.data = f"Hello ROS2 {self.counter}"
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
