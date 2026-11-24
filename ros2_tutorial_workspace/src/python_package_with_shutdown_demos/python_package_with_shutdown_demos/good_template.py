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
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
