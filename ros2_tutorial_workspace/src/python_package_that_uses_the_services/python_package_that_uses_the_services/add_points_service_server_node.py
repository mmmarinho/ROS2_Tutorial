"""
MIT LICENSE

Copyright (C) 2023-25 Murilo Marques Marinho (www.murilomarinho.info)

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
from textwrap import dedent  # https://docs.python.org/3/library/textwrap.html#textwrap.dedent

import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import AddPoints


class AddPointsServiceServerNode(Node):
    """A ROS2 Node with a Service Server for AddPoints."""

    def __init__(self):
        super().__init__('what_is_the_point_service_server')

        self.service_server = self.create_service(
            srv_type=AddPoints,
            srv_name='/add_points',
            callback=self.add_points_service_callback)

        self.service_server_call_count: int = 0

    def add_points_service_callback(self,
                                   request: AddPoints.Request,
                                   response: AddPoints.Response
                                   ) -> AddPoints.Response:
        """
        Adds the two points `a` and `b` in the request and returns the `result`.
        """

        response.result.x = request.a.x + request.b.x
        response.result.y = request.a.y + request.b.y
        response.result.z = request.a.z + request.b.z

        return response


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        add_points_service_server_node = AddPointsServiceServerNode()

        rclpy.spin(add_points_service_server_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
