import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.publisher_ = self.create_publisher(Image, "camera_image", 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        param = self.declare_parameter("camera_id", 0)
        id = self.get_parameter_or(param.name, param).get_parameter_value().integer_value
        assert isinstance(id, int)
        self.cap = cv2.VideoCapture(id)
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()

    def _generate_header(self) -> Header:
        from builtin_interfaces.msg import Time

        now = self.get_clock().now().to_msg()
        assert isinstance(now, Time)
        return Header(frame_id="camera_reader", stamp=now)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8", header=self._generate_header())
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Failed to capture image")


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
