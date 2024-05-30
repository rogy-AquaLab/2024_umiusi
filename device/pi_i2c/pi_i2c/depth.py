import sys

import rclpy
from rclpy.node import Node
from packet_interfaces.msg import Depth


class Depth(Node):
    def __init__(self) -> None:
        super().__init__("depth")
        self._current_publisher = self.create_publisher(Depth, "depth", 10)
        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self) -> None:
        # TODO
        # 深さセンサーからデータを取得してpublish
        self.get_logger().info("tick")


def main(args=sys.argv):
    rclpy.init(args=args)
    depth = Depth()
    rclpy.spin(depth)
    depth.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()