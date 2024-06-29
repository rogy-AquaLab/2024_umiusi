import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.publisher_ = self.create_publisher(Image, "camera_image", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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
