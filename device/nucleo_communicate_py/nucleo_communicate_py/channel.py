# ref:
# https://github.com/ros2/examples/blob/7e47aee/rclpy/executors/examples_rclpy_executors/composed.py

import sys

import rclpy
from packet_interfaces.msg import Current, Flex, Power, Voltage
from rclpy.node import Node
from std_msgs.msg import Empty, Header

from .mutex_serial import MutexSerial
from .receiver import Recv
from .sender import Sndr


class Channel(Node):
    def __init__(self, mutex_serial: MutexSerial):
        super().__init__("channel")
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
        self._sender = Sndr(mutex_serial)
        # Receiver.__init__
        self._current_publisher = self.create_publisher(Current, "current", 10)
        # NOTE: packet_interfaces/Composed の命名と揃える
        self._flex1_publisher = self.create_publisher(Flex, "flex_1", 10)
        self._flex2_publisher = self.create_publisher(Flex, "flex_2", 10)
        self._voltage_publisher = self.create_publisher(Voltage, "voltage", 10)
        self._timer = self.create_timer(0.5, self._recv_callback)
        self._recv = Recv(mutex_serial)

    def _generate_header(self, frame_id: str = "receiver") -> Header:
        from builtin_interfaces.msg import Time

        now = self.get_clock().now().to_msg()
        assert isinstance(now, Time)
        return Header(frame_id=frame_id, stamp=now)

    # Receiver implementation
    def _recv_callback(self):
        self.get_logger().debug("tick")
        flex1, flex2, current, voltage = self._recv.receive_raw()
        self.get_logger().info(
            f"received from nucleo: {flex1=}, {flex2=}, {current=}, {voltage=}",
        )
        flex1, flex2, current, voltage = self._recv.map_values(
            flex1,
            flex2,
            current,
            voltage,
        )
        flex1_msg = Flex(value=flex1, header=self._generate_header("nucleo_flex_1"))
        self._flex1_publisher.publish(flex1_msg)
        flex2_msg = Flex(value=flex2, header=self._generate_header("nucleo_flex_2"))
        self._flex2_publisher.publish(flex2_msg)
        # TODO: データのマッピングはnucleo側と相談
        current_msg = Current(
            value=current, header=self._generate_header("nucleo_current")
        )
        self._current_publisher.publish(current_msg)
        voltage_msg = Voltage(
            value=voltage, header=self._generate_header("nucleo_voltage")
        )
        self._voltage_publisher.publish(voltage_msg)

    # Sender implementations
    def _quit_callback(self, _quit: Empty) -> None:
        self._sender.send_quit()
        self.get_logger().info("Sent quit order")

    def _order_callback(self, order: Power) -> None:
        self._sender.send_power(order)
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
