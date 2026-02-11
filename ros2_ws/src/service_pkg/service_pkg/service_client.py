import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def parse_toggle_value(argv):
    if len(argv) <= 1:
        return True

    value = argv[1].strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False

    raise ValueError(
        "Invalid toggle value. Use true/false (or 1/0, yes/no, on/off)."
    )


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

    try:
        toggle_value = parse_toggle_value(sys.argv)
    except ValueError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2) from exc

    response = node.send_request(toggle_value)
    node.get_logger().info(
        f"Response: success={response.success}, message='{response.message}'"
    )

    node.destroy_node()
    rclpy.shutdown()
