from rclpy.node import Node


class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.create_timer(1.0, self.beat)

    def beat(self):
        self.get_logger().info('alive')
