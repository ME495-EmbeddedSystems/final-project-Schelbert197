import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image

class ImageModification(Node):
    def __init__(self):
        super().__init__("image_modification")

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to get image
        self.cap = self.create_subscription(Image, "camera/color/image_raw", self.image_modification, qos_profile=10)

    def image_modification(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", self.frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageModification()
    rclpy.spin(node)
    rclpy.shutdown()