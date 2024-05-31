import sys

import rclpy
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import Empty
from packet_interfaces.msg import Power

from .mutex_serial import MutexSerial


class Sndr:
    def __init__(self, mutex_serial: MutexSerial) -> None:
        self._mutex_serial = mutex_serial

    def send_power(self, power: Power) -> int:
        buf = bytes([
            # Sending nodification
            0x00,
            # BLDC 1
            (power.bldc[0] >> 0) & 0xFF,
            (power.bldc[0] >> 8) & 0xFF,
            # BLDC 2
            (power.bldc[1] >> 0) & 0xFF,
            (power.bldc[1] >> 8) & 0xFF,
            # BLDC 3
            (power.bldc[2] >> 0) & 0xFF,
            (power.bldc[2] >> 8) & 0xFF,
            # BLDC 4
            (power.bldc[3] >> 0) & 0xFF,
            (power.bldc[3] >> 8) & 0xFF,
            # Servo 1
            (power.servo[0] >> 0) & 0xFF,
            (power.servo[0] >> 8) & 0xFF,
            # Servo 2
            (power.servo[1] >> 0) & 0xFF,
            (power.servo[1] >> 8) & 0xFF,
            # Servo 3
            (power.servo[2] >> 0) & 0xFF,
            (power.servo[2] >> 8) & 0xFF,
            # Servo 4
            (power.servo[3] >> 0) & 0xFF,
            (power.servo[3] >> 8) & 0xFF,
        ])
        with self._mutex_serial.lock() as serial:
            assert isinstance(serial, Serial)
            return serial.write(buf)

    def send_quit(self) -> int:
        buf = bytes([0xFF])
        with self._mutex_serial.lock() as serial:
            assert isinstance(serial, Serial)
            return serial.write(buf)


class Sender(Node):
    def __init__(self, mutex_serial: MutexSerial) -> None:
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
        self._sender = Sndr(mutex_serial)

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
    sender = Sender(mutex_serial)
    rclpy.spin(sender)
    sender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
