import rclpy

from python_package_with_shutdown_demos._heartbeat import Heartbeat


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
