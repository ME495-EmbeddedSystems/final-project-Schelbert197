import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from paddleocr import PaddleOCR

class Ocr(Node):
        def __init__(self):
            super().__init__("ocr")

            self.timer = self.create_timer(1.0/200.0, self.timer_callback)
            self.ocr = PaddleOCR(lang='en', det= False, use_gpu= False) # need to run only once to download and load model into memory
            self.cap = cv2.VideoCapture(0)

        def timer_callback(self):
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            ret, frame = self.cap.read()
            valid = False
            if ret == True:
                # Display image
                cv2.imshow("camera", frame)

                if cv2.waitKey(1) & 0xFF == ord('c'):
                    # while valid == False:
                    #     result = self.ocr.ocr(frame, det=False, cls=False)
                    #     if len(result[0][0][0]) == 1 or len(result[0][0][0]) == 6:
                    #         valid = True
                    #     else
                    result = self.ocr.ocr(frame, det=False, cls=False)
                    if len(result[0][0][0]) == 1 or len(result[0][0][0]) == 6:
                        if result[0][0][1] > 0.75:
                            print(result)
                    # for idx in range(len(result)):
                    #     res = result[idx]
                    #     for line in res:
                    #         print(line)          
                else:
                    cv2.waitKey(1)  
            # Display the message on the console
            # self.get_logger().info('Publishing video frame')
        
        # def image_modification(self):
             

def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
