import rclpy
from rclpy.node import Node


class PrintForeverNode(Node):

    def __init__(self):
        super().__init__('print_forever')
        timer_period: float = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.print_count: int = 0

    def timer_callback(self):
        self.get_logger().info('Printed {} times.'.format(self.print_count))


def main(args=None):
    try:
        rclpy.init(args=args)

        print_forever_node = PrintForeverNode()

        rclpy.spin(print_forever_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
