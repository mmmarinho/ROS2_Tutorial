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
from math import sin, cos, pi
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

import tf2_ros

class TF2BroadcasterNode(Node):
    """A ROS2 Node that broadcasts a TransformStamped in compliance with `tf2_ros2`."""

    def __init__(self):
        super().__init__('tf2_broadcaster_node')

        # Robot name
        self.robot_name = "robot_1"

        # Initialize the transform broadcaster
        ## Note that this object is part of `tf2_ros`, not `rclpy`
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer_period: float = 0.01
        self.timer_elapsed_time: float = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # We define some parameters of our trajectory.
        # Let's make the translation be a circle in 2D with radius of 0.2 meters and frequency of 0.1 Hz.
        # The rotation will be the robot spinning about the z-axis with the same frequency.
        trajectory_radius = 0.2
        trajectory_frequency = 0.1

        # Create an instance of a TransformStamped, used by the broadcaster
        tfs = TransformStamped()

        ## Initialize header with
        # Timestamp equal to the current clock time
        tfs.header.stamp = self.get_clock().now().to_msg()
        # Frame of reference
        tfs.header.frame_id = 'world'
        # The object whose transform this message is about
        tfs.child_frame_id = self.robot_name

        # Set the translation of the transform as a circle in the x-y plane
        tfs.transform.translation.x = trajectory_radius * cos(2 * pi * trajectory_frequency * self.timer_elapsed_time)
        tfs.transform.translation.y = trajectory_radius * sin(2 * pi * trajectory_frequency * self.timer_elapsed_time)
        tfs.transform.translation.z = 0.0

        # Set the rotation (Quaternion) of the transform as a rotation about the z-axis
        phi: float = 2.0 * pi * trajectory_frequency * self.timer_elapsed_time
        tfs.transform.rotation.w = cos(phi/2.0)
        tfs.transform.rotation.x = 0.0
        tfs.transform.rotation.y = 0.0
        tfs.transform.rotation.z = sin(phi/2.0)

        # Send the transformation
        self.transform_broadcaster.sendTransform(tfs)

        # Update internal time counter
        self.timer_elapsed_time += self.timer_period


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = TF2BroadcasterNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()