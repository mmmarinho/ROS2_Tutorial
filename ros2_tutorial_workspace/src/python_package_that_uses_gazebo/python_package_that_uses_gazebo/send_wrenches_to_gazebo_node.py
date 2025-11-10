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
from ros_gz_interfaces.msg import EntityWrench

import rclpy
from rclpy.node import Node

class SendWrenchesToGazeboNode(Node):
    """A ROS2 Node that sends wrenches to Gazebo.

    This Node must be paired with ros_gz_bridge to be able to send wrenches to Gazebo from ROS2.

    This is the equivalent protobuf message sent to Gazebo, considering possible changes in values.

        gz topic -t \
        /world/shapes_with_tf2_and_wrench/wrench \
        -m gz.msgs.EntityWrench \
        -p  'entity: {id: 9}, wrench: {force: {x: 1000.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
    """

    def __init__(self):
        super().__init__('send_wrenches_to_gazebo_node')

        self.entity_wrench_publisher = self.create_publisher(
            msg_type=EntityWrench,
            topic='/world/shapes_with_tf2_and_wrench/wrench',
            qos_profile=1)

        while not self.count_subscribers('/world/shapes_with_tf2_and_wrench/wrench'):
            print(f"Waiting for subscriber to be connected...")
            time.sleep(1)

    def send_wrench_to_gazebo(self):
        """Basic method showing the creation of Wrenches objects."""

        ew = EntityWrench()

        # Add the entity id. The entity.name is currently ignored by Gazebo.
        ew.entity.id = 9

        # Set the force
        ew.wrench.force.x = 1000.0
        ew.wrench.force.y = 0.0
        ew.wrench.force.z = 0.0

        # Set the torque
        ew.wrench.torque.x = 0.0
        ew.wrench.torque.y = 0.0
        ew.wrench.torque.z = 0.0

        self.get_logger().info(f"This sent entity {ew.entity.name} a wrench with force:"
                               f" {ew.wrench.force} "
                               f"and torque:"
                               f" {ew.wrench.torque}.")


        self.entity_wrench_publisher.publish(ew)


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = SendWrenchesToGazeboNode()
        node.send_wrench_to_gazebo()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()
