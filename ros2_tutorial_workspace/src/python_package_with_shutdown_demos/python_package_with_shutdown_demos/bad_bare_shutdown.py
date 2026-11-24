import rclpy
from rclpy.executors import ExternalShutdownException

from python_package_with_shutdown_demos._heartbeat import Heartbeat


def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
