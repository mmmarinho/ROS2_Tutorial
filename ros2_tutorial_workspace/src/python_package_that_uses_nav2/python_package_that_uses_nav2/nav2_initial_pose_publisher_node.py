"""
MIT LICENSE

Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)

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
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Nav2InitialPosePublisherNode(Node):
    """A ROS2 Node that publishes the initial pose for nav2."""

    def __init__(self):
        super().__init__('nav2_initial_pose_publisher_node')

        self._topic = '/initialpose'

        self.publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=self._topic,
            qos_profile=1)

        publisher_count = 0
        while publisher_count < 2:
            publisher_count = self.count_publishers(self._topic)
            print(f"Waiting for publisher to be connected to {self._topic}.")
            print(f"Publisher count is {publisher_count}.")
            time.sleep(1)

    def send_initial_pose_with_covariance(self):
        """Method to create the PoseWithCovarianceStamped."""

        print(f"Publishing pose to topic: {self._topic}.")

        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = self.get_clock().now().to_msg()
        pwcs.header.frame_id = 'map'

        pwcs.pose.pose.position.x = -2.1
        pwcs.pose.pose.position.y = -0.3
        pwcs.pose.pose.position.z = 0.0

        pwcs.pose.pose.orientation.w = 1.0
        pwcs.pose.pose.orientation.x = 0.0
        pwcs.pose.pose.orientation.y = 0.0
        pwcs.pose.pose.orientation.z = 0.0

        pwcs.pose.covariance[0] = 0.25
        pwcs.pose.covariance[7] = 0.25
        pwcs.pose.covariance[35] = 0.06853891909122467

        self.publisher.publish(pwcs)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        node = Nav2InitialPosePublisherNode()
        node.send_initial_pose_with_covariance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
