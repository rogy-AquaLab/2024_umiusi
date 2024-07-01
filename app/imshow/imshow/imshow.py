import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Imshow(Node):
    def __init__(self):
        super().__init__("imshow")
        self.subscription = self.create_subscription(
            Image, "camera_image", self.listener_callback, 10,
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    imshow = Imshow()
    rclpy.spin(imshow)
    imshow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
