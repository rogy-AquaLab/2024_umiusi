import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from smbus2 import SMBus

from .depth import Depth
from .imu import Imu


def main(args=sys.argv):
    rclpy.init(args=args)
    try:
        # TODO: use argument
        bus = SMBus("/dev/i2c-1")
        depth = Depth()
        imu = Imu(bus)

        executor = MultiThreadedExecutor()
        executor.add_node(depth)
        executor.add_node(imu)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
