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


class AmazingQuoteSubscriberNode(Node):
    """A ROS2 Node that receives and AmazingQuote and prints out its info."""

    def __init__(self):
        super().__init__('amazing_quote_subscriber_node')
        self.amazing_quote_subscriber = self.create_subscription(
            msg_type=AmazingQuote,
            topic='/amazing_quote',
            callback=self.amazing_quote_subscriber_callback,
            qos_profile=1)

    def amazing_quote_subscriber_callback(self, msg: AmazingQuote):
        """Method that is called when a new msg is received by the node."""
        
        self.get_logger().info(f"""
        I have received the most amazing of quotes.
        It says
            
               '{msg.quote}'
               
        And was thought by the following genius
            
            -- {msg.philosopher_name}
            
        This latest quote had the id={msg.id}.
        """)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        amazing_quote_subscriber_node = AmazingQuoteSubscriberNode()

        rclpy.spin(amazing_quote_subscriber_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
