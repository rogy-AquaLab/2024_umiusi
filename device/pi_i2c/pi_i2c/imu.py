import sys

import rclpy
import smbus2
from rclpy.node import Node
from rpi_bno055 import BNO055
from sensor_msgs.msg import Imu as ImuMsg


class Imu(Node):
    def __init__(self, bus: smbus2.SMBus | None = None) -> None:
        super().__init__("imu")
        self._bno055 = BNO055(bus=bus)
        self._publisher = self.create_publisher(ImuMsg, "imu", 10)
        self._timer = self.create_timer(0.5, self._timer_callback)

        # https://github.com/H1rono/rpi-bno055/blob/c6516b1920a9d31582977eb1c31f03e68bcf6a5e/rpi_bno055/scripts.py#L115-L128
        self._bno055.begin()
        self._bno055.system_trigger(BNO055.SysTriggerFlag.RST_SYS)
        # accelerometer: m/s^2
        self._bno055.update_unit_selection(BNO055.UnitSelection.ACC_MPS2)
        # gyroscope: rad/s
        self._bno055.update_unit_selection(BNO055.UnitSelection.GYR_RPS)
        # euler: radians
        self._bno055.update_unit_selection(BNO055.UnitSelection.EUL_RADIANS)
        self._bno055.write_mode(BNO055.modes.IMU)

    def _timer_callback(self) -> None:
        # TODO
        # IMUからデータを取得してpublish
        self.get_logger().debug("tick")
        try:
            # https://github.com/H1rono/rpi-bno055/blob/c6516b1920a9d31582977eb1c31f03e68bcf6a5e/rpi_bno055/scripts.py#L131-L134
            linear_accel = self._bno055.read_linear_accel()
            gravity = self._bno055.read_gravity()
            quaternion = self._bno055.read_quaternion()
            angular_velocity = self._bno055.read_gyroscope()
            self.get_logger().info(f"gravity: {gravity}")
            # TODO: covariance の計算
            msg = ImuMsg()
            msg.header.frame_id = "bno055"
        except OSError:
            self.get_logger().error("Failed to read data from IMU")


def main(args=sys.argv):
    rclpy.init(args=args)
    imu = Imu()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
