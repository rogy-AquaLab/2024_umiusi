import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Depth(Node):
    def __init__(self) -> None:
        super().__init__("imu")
        self._current_publisher = self.create_publisher(Imu, "imu", 10)
        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self) -> None:
        # TODO
        # IMUからデータを取得してpublish
        self.get_logger().info("tick")


def main(args=sys.argv):
    rclpy.init(args=args)
    imu = Depth()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
