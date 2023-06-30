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
import rclpy
from rclpy.node import Node
from package_with_interfaces.msg import AmazingQuote


class AmazingQuotePublisherNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('amazing_quote_publisher_node')

        self.amazing_quote_publisher = self.create_publisher(
            msg_type=AmazingQuote,
            topic='/amazing_quote',
            qos_profile=1)

        timer_period: float = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.incremental_id: int = 0

    def timer_callback(self):
        """Method that is periodically called by the timer."""

        amazing_quote = AmazingQuote()
        amazing_quote.id = self.incremental_id
        amazing_quote.quote = 'Use the force, Pikachu!'
        amazing_quote.philosopher_name = 'Uncle Ben'

        self.amazing_quote_publisher.publish(amazing_quote)

        self.incremental_id = self.incremental_id + 1


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        amazing_quote_publisher_node = AmazingQuotePublisherNode()

        rclpy.spin(amazing_quote_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
