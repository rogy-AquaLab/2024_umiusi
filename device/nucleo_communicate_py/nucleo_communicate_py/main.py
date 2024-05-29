# ref:
# https://github.com/ros2/examples/blob/7e47aee/rclpy/executors/examples_rclpy_executors/composed.py

import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from .receiver import Receiver
from .sender import Sender


def main(args=sys.argv):
    rclpy.init(args=args)
    try:
        sender = Sender()
        receiver = Receiver()

        executor = SingleThreadedExecutor()
        executor.add_node(sender)
        executor.add_node(receiver)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
