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
from rclpy.task import Future
from rclpy.node import Node

from package_with_interfaces.srv import WhatIsThePoint


class WhatIsThePointServiceClientNode(Node):
    """A ROS2 Node with a Service Client for WhatIsThePoint."""

    def __init__(self):
        super().__init__('what_is_the_point_service_client')

        self.service_client = self.create_client(
            srv_type=WhatIsThePoint,
            srv_name='/what_is_the_point')

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service {} not available, waiting...'.format(self.service_client.srv_name))

        self.future: Future = None

        timer_period: float = 0.5
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback)

    def timer_callback(self):
        """Method that is periodically called by the timer."""

        request = WhatIsThePoint.Request()
        if random.uniform(0, 1) < 0.5:
            request.quote.quote = "I wonder about the Ultimate Question of Life, the Universe, and Everything."
            request.quote.philosopher_name = "Creators of Deep Thought"
            request.quote.id = 1979
        else:
            request.quote.quote = """[...] your living... it is always potatoes. I dream of potatoes."""
            request.quote.philosopher_name = "a young Maltese potato farmer"
            request.quote.id = 2013

        if self.future is not None and not self.future.done():
            self.get_logger().info("Took too long to process async service call."
                                   "Is the Service Server still alive?")
        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.process_response)

    async def process_response(self, future: Future):
        """Callback for the future, that will be called when it is done"""
        response = future.result()
        if response is not None:
            self.get_logger().info(dedent("""
                We have thus received the point of our quote.

                            {}
            """.format(
                (response.point.x, response.point.y, response.point.z)
            )))
        else:
            self.get_logger().info("""
                    The response was None. :(    
            """)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        what_is_the_point_service_client_node = WhatIsThePointServiceClientNode()

        rclpy.spin(what_is_the_point_service_client_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
