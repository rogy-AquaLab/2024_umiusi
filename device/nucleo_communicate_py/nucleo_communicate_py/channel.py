# ref:
# https://github.com/ros2/examples/blob/7e47aee/rclpy/executors/examples_rclpy_executors/composed.py

import sys

import rclpy
from packet_interfaces.msg import Current, Flex, Power, Voltage
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Empty

from .mutex_serial import MutexSerial
from .receiver import RecvNodeBase, RecvNodeOperator
from .sender import SenderNodeBase, SenderNodeOperator


class Channel(RecvNodeBase, SenderNodeBase):
    def __init__(self, mutex_serial: MutexSerial):
        Node.__init__(self, "channel")
        # Senrder.__init__
        self._quit_subscription = self.create_subscription(
            Empty,
            "quit",
            self._quit_callback,
            10,
        )
        self._order_subscription = self.create_subscription(
            Power,
            "order/power",
            self._order_callback,
            10,
        )
        self._sender_operator = SenderNodeOperator(mutex_serial)
        # Receiver.__init__
        self._current_publisher = self.create_publisher(Current, "current", 10)
        # NOTE: packet_interfaces/Composed の命名と揃える
        self._flex1_publisher = self.create_publisher(Flex, "flex_1", 10)
        self._flex2_publisher = self.create_publisher(Flex, "flex_2", 10)
        self._voltage_publisher = self.create_publisher(Voltage, "voltage", 10)
        self._timer = self.create_timer(0.04, self._recv_callback)
        self._recv_operator = RecvNodeOperator(mutex_serial)

    # Receiver implementation
    @property
    def flex1_publisher(self) -> Publisher:
        return self._flex1_publisher

    @property
    def flex2_publisher(self) -> Publisher:
        return self._flex2_publisher

    @property
    def current_publisher(self) -> Publisher:
        return self._current_publisher

    @property
    def voltage_publisher(self) -> Publisher:
        return self._voltage_publisher

    def _recv_callback(self) -> None:
        self.get_logger().debug("tick")
        self._recv_operator.receive_and_publish(self)

    # Sender implementations
    def _quit_callback(self, _quit: Empty) -> None:
        self._sender_operator.send_quit(self)
        self.get_logger().info("Sent quit order")

    def _order_callback(self, order: Power) -> None:
        self._sender_operator.send_power(self, order)
        self.get_logger().info("Sent power order")


def main(args=sys.argv):
    rclpy.init(args=args)
    # FIXME: receiver.pyのコメント参照
    mutex_serial = MutexSerial("/dev/ttyACM0")
    channel = Channel(mutex_serial)
    rclpy.spin(channel)
    channel.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
