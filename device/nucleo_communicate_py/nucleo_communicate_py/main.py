# ref:
# https://github.com/ros2/examples/blob/7e47aee/rclpy/executors/examples_rclpy_executors/composed.py

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from .mutex_serial import MutexSerial
from .receiver import Receiver
from .sender import Sender


def main(args=sys.argv):
    rclpy.init(args=args)
    # FIXME: receiver.pyのコメント参照
    mutex_serial = MutexSerial("/dev/ttyACM0")
    try:
        sender = Sender(mutex_serial)
        receiver = Receiver(mutex_serial)

        executor = MultiThreadedExecutor()
        executor.add_node(sender)
        executor.add_node(receiver)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
