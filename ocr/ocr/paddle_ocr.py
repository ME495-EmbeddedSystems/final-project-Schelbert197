import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        self.cap_1 = self.create_subscription(Image, "modified_image_1", self.image_reader_1, qos_profile=10)
        self.cap_2 = self.create_subscription(Image, "modified_image_2", self.image_reader_2, qos_profile=10)

        #declare and define parameters
        self.declare_parameter('ocr_frequency', 1)
        self.param_ocr_frequency = self.get_parameter('ocr_frequency').get_parameter_value().integer_value

        # create timer for calling the ocr function
        self.timer = self.create_timer(1.0/self.param_ocr_frequency, self.ocr_timer)

        # Specify the size and type of the empty image
        width, height = 640, 480
        channels = 3
        # Initialize an empty image with the specified size and type
        empty_image = np.zeros((height, width, channels), dtype=np.uint8)

        self.frame_1 = empty_image
        self.frame_2 = empty_image

    def ocr_timer(self):
        """Call the ocr function"""
        self.ocr_func_1(self.frame_1)
        self.ocr_func_2(self.frame_2)

    def ocr_func_1(self, frame):
        """Run OCR on the image frame"""
        result = self.paddle_ocr.ocr(frame, cls=False)
        # if result[0] != None:
        #     self.guess_verification(result)
        print(result)
    
    def ocr_func_2(self, frame):
        """Run OCR on the image frame"""
        result = self.paddle_ocr.ocr(frame, cls=False,  det=False, rec=True)
        # if result[0] != None:
        #     self.guess_verification(result)
        print(result)

    def image_reader_1(self, msg):
        """Convert image to opencv format"""
        self.frame_1 = self.cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow("read_image_1", self.frame_1)
        cv2.waitKey(1)
    
    def image_reader_2(self, msg):
        """Convert image to opencv format"""
        self.frame_2 = self.cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow("read_image_2", self.frame_2)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Paddle_Ocr()
    rclpy.spin(node)
    rclpy.shutdown()