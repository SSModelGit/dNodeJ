from ground_model_interfaces.srv import TwoDimWorldScalarStateReq

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(TwoDimWorldScalarStateReq, 'two_dim_world_state_req', self.state_req_callback)

    def state_req_callback(self, request, response):
        response.state = request.x1 + request.x2
        self.get_logger().info('Incoming request\nx1: %f x2: %f' % (request.x1, request.x2))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()