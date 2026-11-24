import rclpy
from rclpy.executors import ExternalShutdownException

from python_package_with_shutdown_demos._heartbeat import Heartbeat


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = Heartbeat()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
