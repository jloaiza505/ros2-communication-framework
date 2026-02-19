#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def build_toggle_message(enabled):
    return f"Talker enabled: {enabled}"


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')

        self.enabled = True
        self.publisher_ = self.create_publisher(Bool, 'talker_enabled', 10)

        self.srv = self.create_service(
            SetBool,
            'toggle_talker',
            self.handle_toggle
        )

        self.get_logger().info("Service server ready.")

    def handle_toggle(self, request, response):
        self.enabled = request.data
        msg = Bool()
        msg.data = self.enabled
        self.publisher_.publish(msg)

        response.success = True
        response.message = build_toggle_message(self.enabled)

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
