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
import time
from ros_gz_interfaces.srv import SetEntityPose

import rclpy
from rclpy.node import Node

class SendPosesToGazeboNode(Node):
    """A ROS2 Node that sends poses to Gazebo.

    This Node must be paired with ros_gz_bridge to be able to send poses to Gazebo from ROS2.

    """

    def __init__(self):
        super().__init__('send_poses_to_gazebo_node')

        self.service_client = self.create_client(
            srv_type=SetEntityPose,
            srv_name='/world/shapes_with_tf2_and_wrench/set_pose')

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')


    def send_pose_to_gazebo(self):
        """Sample method sending a pose to and entity in Gazebo."""

        request = SetEntityPose.Request()

        # Add the entity id. The entity.name is currently ignored by Gazebo.
        request.entity.id = 9

        # Set the position
        request.pose.position.x = 20.0
        request.pose.position.y = 0.0
        request.pose.position.z = 0.0

        # Set the orientation
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 1.0

        return self.service_client.call_async(request)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = SendPosesToGazeboNode()
        rclpy.spin_until_future_complete(node, node.send_pose_to_gazebo())

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()
