import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from paddleocr import PaddleOCR
import imutils
from imutils.perspective import four_point_transform
from imutils import contours

class Ocr(Node):
        def __init__(self):
            super().__init__("ocr")

            self.timer = self.create_timer(1.0/200.0, self.timer_callback)
            self.ocr = PaddleOCR(lang='en', det= False, use_gpu= False) # need to run only once to download and load model into memory
            self.cap = cv2.VideoCapture(0)

        def timer_callback(self):
            # # Capture frame-by-frame
            # # This method returns True/False as well
            # # as the video frame.
            # ret, frame = self.cap.read()

            # if ret == True:
            #     # Display image
            #     cv2.imshow("camera", frame)

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

            self.image_modification()
            # if frame != None:
            #     result = self.ocr.ocr(frame, det=False, cls=False)
            #     print(result)


        
        def image_modification(self):
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            ret, frame = self.cap.read()

            if ret == True:
                # Display image
                cv2.imshow("frame", frame)

                resized_image = imutils.resize(frame, height=500)
                # cv2.imshow("resized", resized_image)
                gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
                # cv2.imshow("gray", gray)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edged = cv2.Canny(blurred, 50, 200, 255)
                cv2.imshow("edged", edged)

                # find contours in the edge map, then sort them by their
                # size in descending order
                cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
                displayCnt = None

                # loop over the contours
                for c in cnts:
                    # approximate the contour
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                	# if the contour has four vertices, then we have found
                    # the whiteboard
                    if len(approx) == 4:
                        displayCnt = approx
                        break
                # extract the bounded whiteboard region, apply a perspective transform
                # to it
                try:
                    warped = four_point_transform(gray, displayCnt.reshape(4, 2))
                    output = four_point_transform(resized_image, displayCnt.reshape(4, 2))
                    cv2.imshow("transformed", warped)
                except:
                    pass

                cv2.waitKey(1)


             

def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
