"""
MIT LICENSE

Copyright (C) 2023-2025 Murilo Marques Marinho (www.murilomarinho.info)

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
from .add_points_service_server_node import AddPointsServiceServerNode

import rclpy
from rclpy.service import ServiceIntrospectionState
from rclpy.qos import qos_profile_system_default

class AddPointsServiceServerIntrospectionNode(AddPointsServiceServerNode):

    def __init__(self):
        super().__init__()

        # https://github.com/ros2/demos/blob/rolling/demo_nodes_py/demo_nodes_py/services/introspection.py
        self.service_server.configure_introspection(
            clock=self.get_clock(),
            service_event_qos_profile=qos_profile_system_default,
            introspection_state=ServiceIntrospectionState.CONTENTS)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        add_points_service_server_introspection_node = AddPointsServiceServerIntrospectionNode()

        rclpy.spin(add_points_service_server_introspection_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
