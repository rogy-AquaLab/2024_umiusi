import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from .depth import Depth
from .imu import Imu


def main(args=sys.argv):
    rclpy.init(args=args)
    try:
        depth = Depth()
        imu = Imu()

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
