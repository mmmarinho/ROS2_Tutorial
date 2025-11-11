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

import tf2_ros

class ControlShapeThrustNode(Node):
    """A ROS2 Node that controls the thrust of a shape in Gazebo.
       It will apply force based on the position of an entity."""

    def __init__(self):
        super().__init__('control_shape_thurst_node')

        # Structured easily to be changed into configurable parameters
        self._gazebo_world_name = "shapes_with_tf2_and_wrench"
        self._wrench_topic = f'/world/{self._gazebo_world_name}/wrench'
        self._gazebo_entity_name = "box" # Find this in Gazebo
        self._gazebo_entity_id = 9 # Find this in Gazebo

        # Set up the wrench publisher
        self.wrench_publisher = self.create_publisher(
            msg_type=EntityWrench,
            topic=self._wrench_topic,
            qos_profile=1)

        # This is one way to prevent published messages from being lost, but we don't know what is subscribing.
        while not self.count_subscribers(self._wrench_topic):
            print(f"Waiting for subscriber to be connected to {self._wrench_topic}...")
            time.sleep(1)

        # Setting up the TransformListener.
        self.transform_listener_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_listener_buffer, self)

        # Information about the transform we want to listen to
        self.parent_name = self._gazebo_world_name # It will be populated this way by Gazebo
        self.child_name = self._gazebo_entity_name # It will be populated this way by Gazebo

        self.timer_period: float = 0.001
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def compute_control_action(self, current, target, proportional_gain:float = 5.0) -> float:
        """A simple proportional controller"""
        error = current - target
        return -proportional_gain * error

    def send_force_to_gazebo(self, force: tuple[float, float, float] = (0, 0, 0)) -> None:
        """Basic method to send force to an entity."""

        ew = EntityWrench()

        # Add the entity id. The entity.name is currently ignored by Gazebo.
        ew.entity.id = self._gazebo_entity_id

        # Set the force
        ew.wrench.force.x = force[0]
        ew.wrench.force.y = force[1]
        ew.wrench.force.z = force[2]

        # Set the torque
        ew.wrench.torque.x = 0.0
        ew.wrench.torque.y = 0.0
        ew.wrench.torque.z = 0.0

        self.wrench_publisher.publish(ew)

    def timer_callback(self):
        """The timer callback will look up the transforms and send the next control action."""

        try:
            tfs =   self.transform_listener_buffer.lookup_transform(
                    self.parent_name,
                    self.child_name,
                    rclpy.time.Time())

            # A simple proportional controller for the x-axis force
            u = self.compute_control_action(tfs.transform.translation.x, -3.0)

            self.send_force_to_gazebo(
                (u,
                 0.0,
                 0.0)
            )

        except tf2_ros.TransformException as e:

            self.get_logger().warn(
                f'Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}')


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = ControlShapeThrustNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()
