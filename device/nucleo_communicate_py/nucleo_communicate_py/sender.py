import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from packet_interfaces.msg import Power


class Sender(Node):
    def __init__(self) -> None:
        super().__init__("sender")
        self._quit_subscription = self.create_subscription(
            Empty,
            "quit",
            self._quit_callback,
            10
        )
        self._order_subscription = self.create_subscription(
            Power,
            "order/power",
            self._order_callback,
            10
        )

    def _quit_callback(self, _quit: Empty) -> None:
        # TODO: UARTで書き込み
        # https://github.com/rogy-AquaLab/2024_cavolinia_nucleo 参照
        # [0xFF] を送る
        self.get_logger().info("Received quit order")

    def _order_callback(self, _order: Power) -> None:
        # TODO
        # https://github.com/rogy-AquaLab/2024_cavolinia_nucleo 参照
        # [0x00, ...] を送る
        self.get_logger().info("Received power order")


def main(args=sys.argv):
    rclpy.init(args=args)
    sender = Sender()
    rclpy.spin(sender)
    sender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
