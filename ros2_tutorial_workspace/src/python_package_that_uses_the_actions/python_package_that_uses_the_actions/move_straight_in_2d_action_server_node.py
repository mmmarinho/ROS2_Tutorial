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
from math import sqrt

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from geometry_msgs.msg import Point
from package_with_interfaces.action import MoveStraightIn2D


class MoveStraightIn2DActionServerNode(Node):
    """A ROS2 Node with an Action Server for MoveStraightIn2D."""

    def __init__(self):
        super().__init__('move_straight_in_2d_action_server')
        self.current_position = Point()
        self.MAX_ITERATIONS: int  = 100
        self.sampling_time: float = 0.01

        self.action_server = ActionServer(
            self,
            MoveStraightIn2D,
            '/move_straight_in_2d',
            self.execute_callback)



    def get_distance(self, desired_position: Point) -> float:
        """
        Calculates the error norm (e.g. Euclidean distance) between the current position and the desired position.
        Notice that we have chosen to ignore the z-axis as this is a 2D motion.
        """

        x = self.current_position.x
        y = self.current_position.y
        xd = desired_position.x
        yd = desired_position.y

        return sqrt((x - xd) ** 2 + (y - yd) ** 2)

    def move_the_object_with_velocity(self, desired_position: Point, desired_speed: float = 1.0) -> None:
        """
        Moves the object with the desired speed for one iteration. Must be called until objective is reached or
        the controller times out.
        """
        distance = self.get_distance(desired_position)
        # Prevent us from dividing by zero or a small number we currently do not care about
        if distance < 0.01:
            return

        x_direction = (self.current_position.x - desired_position.x) / distance
        y_direction = (self.current_position.y - desired_position.y) / distance

        # Apply new position based on the desired velocity and direction
        self.current_position.x -= x_direction * desired_speed * self.sampling_time
        self.current_position.y -= y_direction * desired_speed * self.sampling_time


    def execute_callback(self, goal: ServerGoalHandle) -> MoveStraightIn2D.Result:
        """
        To be attached to the Action as its callback. Receives a goal position and tries to move to that position.
        It will return the Euclidean distance between the current position and the desired position as the feedback.
        If the goal is reached within a given threshold it will succeed, otherwise it will abort.
        """
        desired_position = goal.request.desired_position

        self.get_logger().info(f'current_position is {self.current_position}.')
        self.get_logger().info(f'desired_position set to {desired_position}.')

        feedback_msg = MoveStraightIn2D.Feedback()

        # Let's limit the maximum number of iterations this can accept
        for i in range(self.MAX_ITERATIONS):
            distance = self.get_distance(desired_position)
            feedback_msg.distance = distance
            goal.publish_feedback(feedback_msg)

            self.move_the_object_with_velocity(desired_position)

            # We define a threshold to see if it managed to reach the goal or not.
            if distance < 0.01:
                goal.succeed()
                break

            # A sleep illustrating the time it would take in real life for a robot to move
            time.sleep(self.sampling_time)

        result = MoveStraightIn2D.Result()
        result.final_position = self.current_position
        return result


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
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
