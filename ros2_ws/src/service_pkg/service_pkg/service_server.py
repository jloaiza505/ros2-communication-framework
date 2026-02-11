import rclpy
from rclpy.node import Node
from service_pkg.srv import ToggleMessage


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')

        self.enabled = True

        self.srv = self.create_service(
            ToggleMessage,
            'toggle_talker',
            self.handle_toggle
        )

        self.get_logger().info("Service server ready.")

    def handle_toggle(self, request, response):
        self.enabled = request.toggle
        response.status = f"Talker enabled: {self.enabled}"

        self.get_logger().info(response.status)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
