import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from paddleocr import PaddleOCR

from sensor_msgs.msg import Image
from std_msgs.msg import String

class Paddle_Ocr(Node):
    def __init__(self):
        super().__init__("paddle_ocr")

        # initialize paddleocr class
        self.paddle_ocr = PaddleOCR(lang='en', use_gpu= False) # need to run only once to download and load model into memory

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to get image
        self.cap = self.create_subscription(Image, "modified_image", self.image_reader, qos_profile=10)

    def image_reader(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow("read_image", self.frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Paddle_Ocr()
    rclpy.spin(node)
    rclpy.shutdown()