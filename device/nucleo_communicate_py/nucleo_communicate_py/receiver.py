import sys

import rclpy
from rclpy.node import Node
from packet_interfaces.msg import Current, Flex, Voltage


class Receiver(Node):
    def __init__(self) -> None:
        super().__init__("receiver")
        self._current_publisher = self.create_publisher(Current, "current", 10)
        # NOTE: packet_interfaces/Composed の命名と揃える
        self._flex1_publisher = self.create_publisher(Flex, "flexsensor1", 10)
        self._flex2_publisher = self.create_publisher(Flex, "flexsensor2", 10)
        self._voltage_publisher = self.create_publisher(Voltage, "voltage", 10)
        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self) -> None:
        # TODO
        # https://github.com/rogy-AquaLab/2024_cavolinia_nucleo 参照
        # UARTでNucleoからデータを取得してpublishする
        self.get_logger().info("tick")


def main(args=sys.argv):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
