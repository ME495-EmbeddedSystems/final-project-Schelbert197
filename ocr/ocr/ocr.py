import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from paddleocr import PaddleOCR
import imutils
from imutils.perspective import four_point_transform
from imutils import contours

from sensor_msgs.msg import Image
from std_msgs.msg import String

class Ocr(Node):
    def __init__(self):
        super().__init__("ocr")

        self.ocr = PaddleOCR(lang='en', use_gpu= False) # need to run only once to download and load model into memory
        
        self.guess_publish = self.create_publisher(String, "user_input", 10)
        self.cv_bridge = CvBridge()
        
        # self.cap = cv2.VideoCapture(0)
        
        self.cap = self.create_subscription(Image, "camera/color/image_raw", self.image_modification, qos_profile=10)
        
        #declare and define parameters
        self.declare_parameter('ocr_frequency', 1)
        self.param_ocr_frequency = self.get_parameter('ocr_frequency').get_parameter_value().integer_value

        self.count = 1
        self.frame = None
        self.guess_tracker = []
        self.guess_pub_tracker = []

        self.timer = self.create_timer(1.0/self.param_ocr_frequency, self.ocr_timer)

    def ocr_timer(self):
        self.ocr_func(self.frame)

    def ocr_func(self, frame):
        result = self.ocr.ocr(frame, cls=False)
        if result[0] != None:
            self.guess_verification(result)
        print(result)
        # self.get_logger().info(f"Result: {result}")

    def guess_verification(self, result):
        if len(result[0][0][1][0]) == 1 or len(result[0][0][1][0]) == 6:
            if result[0][0][1][1] > 0.85:
                self.guess_tracking(result[0][0][1][0])
                # self.get_logger().info(f"Registering Guess: {result[0][0][1][0]}")

    def guess_tracking(self, guess):
        self.guess_tracker.append(guess)
        if len(self.guess_tracker) > 3:
            del self.guess_tracker[0]
        if len(self.guess_tracker) == 3:
            if self.guess_tracker[0] == self.guess_tracker[1] and self.guess_tracker[1] == self.guess_tracker[2]:
                self.get_logger().info(f"Registering Guess: {guess}")
                self.guess_tracker = []
                self.guess_publisher(guess)

    def guess_publisher(self, guess):
        current_guess = String()
        publish = True
        if len(self.guess_pub_tracker) > 0:
            for item in self.guess_pub_tracker:
                if item == guess:
                    publish = False
        if publish:
            self.guess_pub_tracker.append(guess)
            current_guess.data = guess
            self.guess_publish.publish(current_guess)


    def image_modification(self, msg):
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", self.frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
