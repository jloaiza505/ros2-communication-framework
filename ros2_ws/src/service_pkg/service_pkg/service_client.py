import rclpy
from rclpy.node import Node
from service_pkg.srv import ToggleMessage
import sys


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')

        self.cli = self.create_client(ToggleMessage, 'toggle_talker')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, toggle_value):
        req = ToggleMessage.Request()
        req.toggle = toggle_value

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()

    toggle_value = True
    if len(sys.argv) > 1:
        toggle_value = sys.argv[1].lower() == "true"

    response = node.send_request(toggle_value)
    node.get_logger().info(f"Response: {response.status}")

    node.destroy_node()
    rclpy.shutdown()
