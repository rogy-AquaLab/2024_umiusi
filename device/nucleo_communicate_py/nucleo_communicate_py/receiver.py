import abc
import sys

import rclpy
from packet_interfaces.msg import Current, Flex, Voltage
from rclpy.node import Node
from rclpy.publisher import Publisher
from serial import Serial
from std_msgs.msg import Header

from .mutex_serial import MutexSerial


class RecvNodeBase(Node, metaclass=abc.ABCMeta):
    @property
    @abc.abstractmethod
    def flex1_publisher(self) -> Publisher: ...

    @property
    @abc.abstractmethod
    def flex2_publisher(self) -> Publisher: ...

    @property
    @abc.abstractmethod
    def current_publisher(self) -> Publisher: ...

    @property
    @abc.abstractmethod
    def voltage_publisher(self) -> Publisher: ...


class RecvNodeOperator:
    def __init__(self, mutex_serial: MutexSerial) -> None:
        self._mutex_serial = mutex_serial

    # (flex1, flex2, current, voltage)
    def _receive_raw(self) -> tuple[int, int, int, int]:
        with self._mutex_serial.lock() as serial:
            assert isinstance(serial, Serial)
            # Request sensor's data
            serial.write(b"\x01")
            # Receive nucleo's response
            buf = serial.read(8)
        # raw values
        # FIXME: int.from_bytes使ったほうがいいかも
        flex1 = (buf[0] << 0) | (buf[1] << 8)
        flex2 = (buf[2] << 0) | (buf[3] << 8)
        current = (buf[4] << 0) | (buf[5] << 8)
        voltage = (buf[6] << 0) | (buf[7] << 8)
        return (flex1, flex2, current, voltage)

    def _map_values(
        self,
        flex1: int,
        flex2: int,
        current: int,
        voltage: int,
    ) -> tuple[int, int, float, float]:
        # TODO: current, voltageの計算はnucleo側と要相談
        return (flex1, flex2, current / 0xFFFF, voltage / 0xFFFF)

    def _receive_with_node(self, node: RecvNodeBase) -> tuple[int, int, float, float]:
        flex1, flex2, current, voltage = self._receive_raw()
        node.get_logger().debug(
            f"received from nucleo: {flex1=}, {flex2=}, {current=}, {voltage=}",
        )
        return self._map_values(flex1, flex2, current, voltage)

    def _generate_header(self, node: RecvNodeBase, frame_id: str = "receiver") -> Header:
        from builtin_interfaces.msg import Time

        now = node.get_clock().now().to_msg()
        assert isinstance(now, Time)
        return Header(frame_id=frame_id, stamp=now)

    def receive_and_publish(self, node: RecvNodeBase) -> None:
        from functools import partial

        flex1, flex2, current, voltage = self._receive_with_node(node)

        gen_header = partial(self._generate_header, node)
        flex1_msg = Flex(value=flex1, header=gen_header("nucleo_flex_1"))
        flex2_msg = Flex(value=flex2, header=gen_header("nucleo_flex_2"))
        current_msg = Current(value=current, header=gen_header("nucleo_current"))
        voltage_msg = Voltage(value=voltage, header=gen_header("nucleo_voltage"))

        node.flex1_publisher.publish(flex1_msg)
        node.flex2_publisher.publish(flex2_msg)
        node.current_publisher.publish(current_msg)
        node.voltage_publisher.publish(voltage_msg)


class Receiver(RecvNodeBase):
    def __init__(self, mutex_serial: MutexSerial) -> None:
        super().__init__("receiver")
        self._current_publisher = self.create_publisher(Current, "current", 10)
        # NOTE: packet_interfaces/Composed の命名と揃える
        self._flex1_publisher = self.create_publisher(Flex, "flex_1", 10)
        self._flex2_publisher = self.create_publisher(Flex, "flex_2", 10)
        self._voltage_publisher = self.create_publisher(Voltage, "voltage", 10)
        self._timer = self.create_timer(0.04, self._timer_callback)
        self._operator = RecvNodeOperator(mutex_serial)

    # override
    @property
    def flex1_publisher(self) -> Publisher:
        return self._flex1_publisher

    # override
    @property
    def flex2_publisher(self) -> Publisher:
        return self._flex2_publisher

    # override
    @property
    def current_publisher(self) -> Publisher:
        return self._current_publisher

    # override
    @property
    def voltage_publisher(self) -> Publisher:
        return self._voltage_publisher

    def _timer_callback(self) -> None:
        self.get_logger().debug("tick")
        self._operator.receive_and_publish(self)


def main(args=sys.argv):
    rclpy.init(args=args)
    # FIXME: 手元のRaspberryPiだとこれで動くが...?
    mutex_serial = MutexSerial("/dev/ttyACM0")
    receiver = Receiver(mutex_serial)
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
