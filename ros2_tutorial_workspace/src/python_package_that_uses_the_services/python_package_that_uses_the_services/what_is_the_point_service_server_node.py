"""
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

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
from package_with_interfaces.srv import WhatIsThePoint


class WhatIsThePointServiceServerNode(Node):
    """A ROS2 Node with a Service Server for WhatIsThePoint."""

    def __init__(self):
        super().__init__('what_is_the_point_service_server')

        self.service_server = self.create_service(
            srv_type=WhatIsThePoint,
            srv_name='/what_is_the_point',
            callback=self.what_is_the_point_service_callback)

        self.service_server_call_count: int = 0

    def what_is_the_point_service_callback(self,
                                           request: WhatIsThePoint.Request,
                                           response: WhatIsThePoint.Response
                                           ) -> WhatIsThePoint.Response:
        """Analyses an AmazingQuote and returns what is the point.
           If the quote contains 'life', it returns a point whose sum of coordinates is 42.
           Otherwise, it returns a random point whose sum of coordinates is not 42.
        """

        # Generate the x,y,z of the point
        if "life" in request.quote.quote.lower():
            x: float = random.uniform(0, 42)
            y: float = random.uniform(0, 42 - x)
            z: float = 42 - (x + y)
        else:
            x: float = random.uniform(0, 100)
            y: float = random.uniform(0, 100)
            z: float = random.uniform(0, 100)
            if x + y + z == 42:  # So you’re telling me there’s a chance? Yes!
                x = x + 1  # Not anymore :(

        # Assign to the response
        response.point.x = x
        response.point.y = y
        response.point.z = z

        # Increase the call count
        self.service_server_call_count = self.service_server_call_count + 1

        self.get_logger().info(dedent("""
            This is the call number {} to this Service Server.
            The analysis of the AmazingQuote below is complete.
            
                    {}
            
            -- {}
            
            The point has been sent back to the client.
        """.format(
            self.service_server_call_count,
            request.quote.quote,
            request.quote.philosopher_name
        )))

        return response


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        what_is_the_point_service_server_node = WhatIsThePointServiceServerNode()

        rclpy.spin(what_is_the_point_service_server_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
