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

        # create publisher to publish modified image
        self.modified_image_publish = self.create_publisher(Image, "modified_image", 10)

    def image_modification(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", self.frame)
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("gray", gray)

        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        # cv2.imshow("blurred", blurred)

        ret3,th3 = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.imshow("binarized", th3)

        img_publish = self.cv_bridge.cv2_to_imgmsg(th3)
        self.modified_image_publish.publish(img_publish)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageModification()
    rclpy.spin(node)
    rclpy.shutdown()