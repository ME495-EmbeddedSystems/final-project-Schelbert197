import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Ocr(Node):
        def __init__(self):
            super().__init__("ocr")

            self.timer = self.create_timer(1.0/200.0, self.timer_callback)

            self.cap = cv2.VideoCapture(0)

        def timer_callback(self):
            """
            Callback function.
            This function gets called every 0.1 seconds.
            """
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            ret, frame = self.cap.read()
                
            if ret == True:
                # Display image
                cv2.imshow("camera", frame)
                
                cv2.waitKey(1)  
            # Display the message on the console
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
