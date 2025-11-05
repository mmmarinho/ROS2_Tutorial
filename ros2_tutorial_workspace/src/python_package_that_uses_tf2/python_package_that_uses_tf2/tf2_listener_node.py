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
import rclpy
from rclpy.node import Node

import tf2_ros

class TF2ListenerNode(Node):
    """A ROS2 Node that listens to the `tf` topic in compliance with `tf2_ros2`."""

    def __init__(self):
        super().__init__('tf2_listener_node')

        # Setting up the TransformListener
        self.transform_listener_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_listener_buffer, self)

        # Information about the transform we want to listen to
        self.parent_name = "world"
        self.child_name = "robot_1"

        self.timer_period: float = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):

        try:
            tfs =   self.transform_listener_buffer.lookup_transform(
                    self.parent_name,
                    self.child_name,
                    rclpy.time.Time())

            self.get_logger().info(f"Transform: {tfs}")

        except tf2_ros.TransformException as e:

            self.get_logger().error(
                f'Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}')


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = TF2ListenerNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()