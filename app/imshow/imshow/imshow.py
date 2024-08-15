import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class Imshow(Node):
    def __init__(self):
        super().__init__("imshow")
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST,depth=10)
        self.subscription = self.create_subscription(
            CompressedImage,
            "camera_image",
            self.listener_callback,
            qos_profile,
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # CompressedImageデータをNumPy配列に変換
        np_arr = np.frombuffer(msg.data, np.uint8)
        # NumPy配列から画像をデコード
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
        else:
            self.get_logger().error("Failed to decode image")


def main(args=None):
    rclpy.init(args=args)
    imshow = Imshow()
    rclpy.spin(imshow)
    imshow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
