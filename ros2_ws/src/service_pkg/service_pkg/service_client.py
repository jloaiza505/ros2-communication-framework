import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')

        self.cli = self.create_client(SetBool, 'toggle_talker')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, toggle_value):
        req = SetBool.Request()
        req.data = toggle_value

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
    node.get_logger().info(
        f"Response: success={response.success}, message='{response.message}'"
    )

    node.destroy_node()
    rclpy.shutdown()
