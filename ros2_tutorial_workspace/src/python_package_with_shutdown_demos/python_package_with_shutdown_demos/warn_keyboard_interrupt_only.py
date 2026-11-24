import rclpy

from python_package_with_shutdown_demos._heartbeat import Heartbeat


def main(args=None):
    try:
        rclpy.init(args=args)
        node = Heartbeat()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
