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

            self.timer = self.create_timer(1.0/self.param_ocr_frequency, self.ocr_timer)
        def ocr_timer(self):
            # # Capture frame-by-frame
            # # This method returns True/False as well
            # # as the video frame.
            # ret, frame = self.cap.read()

            # if ret == True:
            #     # Display image
            #     cv2.imshow("camera", frame)
            #     cv2.waitKey(1)  


            #     if cv2.waitKey(1) & 0xFF == ord('c'):
            #         # while valid == False:
            #         #     result = self.ocr.ocr(frame, det=False, cls=False)
            #         #     if len(result[0][0][0]) == 1 or len(result[0][0][0]) == 6:
            #         #         valid = True
            #         #     else
            #         result = self.ocr.ocr(frame, det=False, cls=False)
            #         if len(result[0][0][0]) == 1 or len(result[0][0][0]) == 6:
            #             if result[0][0][1] > 0.75:
            #                 print(result)
            #         # for idx in range(len(result)):
            #         #     res = result[idx]
            #         #     for line in res:
            #         #         print(line)          
            #     else:
            #         cv2.waitKey(1)  
            # # Display the message on the console
            # # self.get_logger().info('Publishing video frame')

            # if  self.count%10 == 0:
            #     result = self.ocr.ocr(self.frame, cls=False)
            #     if result[0] != None:
            #         self.guess_verification(result)
            #     print(result)

            # if self.count%100 == 0:
            #     self.ocr_func(self.frame)

            # self.count += 1

            self.ocr_func(self.frame)

        def ocr_func(self, frame):
            # try:
            #     result = self.ocr.ocr(frame, cls=False)
            #     if result[0] != None:
            #         self.guess_verification(result)
            #     # print(result)
            #     # self.get_logger().info(f"Result: {result}")
            # except:
            #     pass
            result = self.ocr.ocr(frame, cls=False)
            if result[0] != None:
                self.guess_verification(result)
            # print(result)
            # self.get_logger().info(f"Result: {result}")

        def guess_verification(self, result):
            if len(result[0][0][1][0]) == 1 or len(result[0][0][1][0]) == 6:
                self.get_logger().info(f"Registering Guess: {result[0][0][1][0]}")

        def image_modification(self, msg):
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("image", self.frame)
            cv2.waitKey(1)

            # if  self.count%100 == 0:
            #     result = self.ocr.ocr(frame, cls=False)
            #     if result[0] != None:
            #         self.guess_verification(result)
            #     print(result)
           
            # self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
