import sys

from ground_model_interfaces.srv import TwoDimWorldScalarStateReq
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(TwoDimWorldScalarStateReq, 'two_dim_world_state_req')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TwoDimWorldScalarStateReq.Request()

    def send_request(self, x, y):
        self.req.x = x
        self.req.y = y
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(float(sys.argv[1]), float(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of two_dim_world_state_req: for %d + %d = %d' %
        (float(sys.argv[1]), float(sys.argv[2]), response.state))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()