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
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point
from package_with_interfaces.action import MoveStraightIn2D

class MoveStraightIn2DActionClientNode(Node):
    """A ROS2 Node with an Action Client for MoveStraightIn2D."""

    def __init__(self):
        super().__init__('move_straight_in_2d_action_client')
        self.action_client = ActionClient(self, MoveStraightIn2D, '/move_straight_in_2d')

    def send_goal(self, desired_position: Point):
        goal_msg = MoveStraightIn2D.Goal()
        goal_msg.desired_position = desired_position

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'service {self.action_client} not available, waiting...')

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.action_feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info(f'The class of the future.result() is {goal_handle.__class__}')

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the server.')
            return
        self.get_logger().info('Goal was accepted by the server.')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.action_result_callback)

    def action_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.final_position}')

    def action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.distance}')


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = MoveStraightIn2DActionClientNode()

        # Send the goal once and then do nothing until the user shuts this node down.
        desired_position = Point()
        desired_position.x = 1.0
        desired_position.y = -1.0
        node.send_goal(desired_position)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()