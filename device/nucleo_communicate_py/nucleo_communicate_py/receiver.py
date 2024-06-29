import sys

import rclpy
from packet_interfaces.msg import Current, Flex, Voltage
from rclpy.node import Node
from serial import Serial

from .mutex_serial import MutexSerial


class Recv:
    def __init__(self, mutex_serial: MutexSerial) -> None:
        self._mutex_serial = mutex_serial

    # (flex1, flex2, current, voltage)
    def receive_raw(self) -> tuple[int, int, int, int]:
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

    def map_values(
        self,
        flex1: int,
        flex2: int,
        current: int,
        voltage: int,
    ) -> tuple[int, int, float, float]:
        # TODO: current, voltageの計算はnucleo側と要相談
        return (flex1, flex2, current / 0xFFFF, voltage / 0xFFFF)

    def receive(self) -> tuple[int, int, float, float]:
        flex1, flex2, current, voltage = self.receive_raw()
        return self.map_values(flex1, flex2, current, voltage)


class Receiver(Node):
    def __init__(self, mutex_serial: MutexSerial) -> None:
        super().__init__("receiver")
        self._current_publisher = self.create_publisher(Current, "current", 10)
        # NOTE: packet_interfaces/Composed の命名と揃える
        self._flex1_publisher = self.create_publisher(Flex, "flex/1", 10)
        self._flex2_publisher = self.create_publisher(Flex, "flex/2", 10)
        self._voltage_publisher = self.create_publisher(Voltage, "voltage", 10)
        self._timer = self.create_timer(0.5, self._timer_callback)
        self._recv = Recv(mutex_serial)

    def _timer_callback(self) -> None:
        self.get_logger().debug("tick")
        flex1, flex2, current, voltage = self._recv.receive_raw()
        self.get_logger().info(
            f"received from nucleo: {flex1=}, {flex2=}, {current=}, {voltage=}",
        )
        flex1, flex2, current, voltage = self._recv.map_values(
            flex1, flex2, current, voltage,
        )
        self._flex1_publisher.publish(Flex(value=flex1))
        self._flex2_publisher.publish(Flex(value=flex2))
        # TODO: データのマッピングはnucleo側と相談
        self._current_publisher.publish(Current(value=current))
        self._voltage_publisher.publish(Voltage(value=voltage))


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
