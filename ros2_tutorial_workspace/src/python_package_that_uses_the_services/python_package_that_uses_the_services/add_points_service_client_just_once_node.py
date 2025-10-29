"""
MIT LICENSE

Copyright (C) 2023-2025 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import random

import rclpy
from rclpy.task import Future
from rclpy.node import Node

from package_with_interfaces.srv import AddPoints


class AddPointsServiceClientJustOnceNode(Node):
    """A ROS2 Node with a Service Client for AddPoints, just once."""

    def __init__(self):
        super().__init__('add_points_service_client_just_once')

        self.service_client = self.create_client(
            srv_type=AddPoints,
            srv_name='/add_points')

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')

        # Create the future in the correct scope. It needs to be an attribute of the instance.
        self.future: Future = None

        # Create service request
        request = AddPoints.Request()

        request.a.x = random.uniform(0, 1000)
        request.a.y = random.uniform(0, 1000)
        request.a.y = random.uniform(0, 1000)

        request.b.x = random.uniform(0, 1000)
        request.b.y = random.uniform(0, 1000)
        request.b.z = random.uniform(0, 1000)

        # Call the service async and set the done callback
        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.process_response)

    def process_response(self, future: Future):
        """Callback for the future, that will be called when it is done."""
        response = future.result()
        if response is not None:
            self.get_logger().info(f"The result was {(response.result.x, response.result.y, response.result.z)}")
        else:
            self.get_logger().info("The response was None.")


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = AddPointsServiceClientJustOnceNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
