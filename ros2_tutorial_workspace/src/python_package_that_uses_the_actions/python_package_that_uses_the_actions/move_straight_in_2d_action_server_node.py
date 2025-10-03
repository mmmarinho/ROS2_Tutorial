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
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Point
from package_with_interfaces.action import MoveStraightIn2D


class MoveStraightIn2DActionServerNode(Node):
    """A ROS2 Node with an Action Server for MoveStraightIn2D."""

    def __init__(self):
        super().__init__('move_straight_in_2d_action_server')

        self.current_position = Point()

        self.action_server = ActionServer(
            self,
            MoveStraightIn2D,
            'move_straight_in_2d',
            self.execute_callback)

    def get_error_norm(self, desired_position):
        print("TODO")

    def execute_callback(self, goal: MoveStraightIn2D.ActionGoalHandle) -> MoveStraightIn2D.Result:
        desired_position = goal.request.desired_position

        self.get_logger().info(f'Target set to {desired_position}...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal.succeed()

        result = MoveStraightIn2D.Result()
        result.final_position = self.current_position
        return result


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        node = MoveStraightIn2DActionServerNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()