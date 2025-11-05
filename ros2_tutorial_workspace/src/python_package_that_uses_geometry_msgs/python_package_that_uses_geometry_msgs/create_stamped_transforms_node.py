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
from math import cos, sin, pi
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

class CreateStampedTransformsNode(Node):
    """A ROS2 Node that creates TransformStamped objects."""

    def __init__(self):
        super().__init__('create_stamped_transforms_node')

    def create_and_print_stamped_transform(self):
        """Basic method showing the creation of TransformStamped objects."""

        tfs = TransformStamped()

        # Initialize header with
        ## Timestamp equal to the current clock time
        tfs.header.stamp = self.get_clock().now().to_msg()
        ## Frame of reference equals `world`
        tfs.header.frame_id = 'world'
        ## The transform will be of the frame will be of this tag
        tfs.child_frame_id = "object_1"

        # Set the translation of the transform as a circle in the x-y plane
        tfs.transform.translation.x = 1.0
        tfs.transform.translation.y = 2.0
        tfs.transform.translation.z = 3.0

        # Set the rotation (Quaternion) of the transform as a rotation about the x-axis
        tfs.transform.rotation.w = cos(pi / 2.0)
        tfs.transform.rotation.x = sin(pi / 2.0)
        tfs.transform.rotation.y = 0.0
        tfs.transform.rotation.z = 0.0

        self.get_logger().info(f"This transform has translation:"
                               f" {tfs.transform.translation} "
                               f"and rotation:"
                               f" {tfs.transform.rotation}.")

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = CreateStampedTransformsNode()
        node.create_and_print_stamped_transform()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()