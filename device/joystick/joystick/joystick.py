import sys

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        self._timer = self.create_timer(0.5, self._timer_callback)
        self._joy_publisher = self.create_publisher(Joy, "joystick", 10)
        # TODO: ここparameterで変えられるようにしたい
        self._joystick = pygame.joystick.Joystick(0)
        # logging
        name = self._joystick.get_name()
        self.get_logger().info(f"Using controller: {name}")

    def _timer_callback(self):
        self.get_logger().info("tick")
        pygame.event.pump()

        numaxes = self._joystick.get_numaxes()
        numbuttons = self._joystick.get_numbuttons()
        self.get_logger().info(f"Found {numaxes} axes and {numbuttons} buttons")

        axes = [self._joystick.get_axis(i) for i in range(numaxes)]
        buttons = [int(self._joystick.get_button(i)) for i in range(numbuttons)]
        msg = Joy(axes=axes, buttons=buttons)
        self._joy_publisher.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    pygame.init()
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    pygame.quit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
