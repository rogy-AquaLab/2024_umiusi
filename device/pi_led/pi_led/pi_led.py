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
        param_r = self.declare_parameter("led_pin_r", "")
        param_g = self.declare_parameter("led_pin_g", "")
        param_b = self.declare_parameter("led_pin_b", "")

        pin_r = (
            self.get_parameter_or(param_r.name, param_r)
            .get_parameter_value()
            .string_value
        )
        pin_g = (
            self.get_parameter_or(param_g.name, param_g)
            .get_parameter_value()
            .string_value
        )
        pin_b = (
            self.get_parameter_or(param_b.name, param_b)
            .get_parameter_value()
            .string_value
        )

        self.led_r = LED(pin_r)
        self.led_g = LED(pin_g)
        self.led_b = LED(pin_b)

    def led_callback(self, light: LedColor):
        self.led_light(self.led_r, light.red)
        self.led_light(self.led_g, light.green)
        self.led_light(self.led_b, light.blue)
        self.get_logger().info(
            f'R is "{int(light.red)}",G is "{int(light.green)}",B is "{int(light.blue)}"'
        )

    def led_light(self, led, light):
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
