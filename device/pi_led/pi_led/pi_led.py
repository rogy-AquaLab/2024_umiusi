import rclpy
from gpiozero import LED
from packet_interfaces.msg import LedColor
from rclpy.node import Node


class Led(Node):
    def __init__(self):
        super().__init__("led")
        self._led_subscription = self.create_subscription(
            LedColor, "led_color", self.led_callback, 10
        )
        self.led_r = LED(17)  # parameter?
        self.led_g = LED(27)
        self.led_b = LED(22)

    def led_callback(self, light: LedColor):
        self.led_light(self.led_r, light.red)
        self.led_light(self.led_g, light.green)
        self.led_light(self.led_b, light.blue)
        self.get_logger().info(
            'R is "%d",G is "%d",B is "%d"' % (self.led_R, self.led_G, self.led_B)
        )

    def led_light(led, light):
        if light:
            led.on()
        else:
            led.off()


def main(args=None):
    rclpy.init(args=args)
    led_subscriber = Led()
    rclpy.spin(led_subscriber)
    led_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
