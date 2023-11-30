import rclpy
from rclpy.node import Node
import numpy as np
import imutils

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image

def nothing(x):
    pass

class ImageModification(Node):
    def __init__(self):
        super().__init__("image_modification")

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to get image
        self.cap = self.create_subscription(Image, "camera/color/image_raw", self.image_modification, qos_profile=10)

        # create publisher to publish modified image
        self.modified_image_1_publish = self.create_publisher(Image, "modified_image_1", 10)
        self.modified_image_2_publish = self.create_publisher(Image, "modified_image_2", 10)

        cv2.namedWindow('Parameters')
        cv2.createTrackbar('Canny_T_min', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar('Canny_T_max', 'Parameters', 0, 255, nothing)
        cv2.setTrackbarPos('Canny_T_max', 'Parameters', 255)


    def image_modification(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2.imshow("image", self.frame)
        resized_image = imutils.resize(self.frame, height=500)
        # cv2.imshow("resized_image", resized_image)
        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        cv2.imshow("blurred", blurred)
        ret3,binarised = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # cv2.imshow("binarized", binarised)
        c_min = cv2.getTrackbarPos('Canny_T_min', 'Parameters')
        c_max = cv2.getTrackbarPos('Canny_T_max', 'Parameters')
        edged = cv2.Canny(blurred, c_min, c_max)
        cv2.imshow("edged", edged)

        # kernel = np.ones((13,13),np.uint8)
        # dilation = cv2.dilate(binarised,kernel,iterations = 1)
        # inverted_image = cv2.bitwise_not(dilation)
        # # cv2.imshow("inverted_image", inverted_image)
        # resized_image = imutils.resize(inverted_image, width=500, height=500)
        # cv2.imshow("resized_image", resized_image)

        # img_publish_1 = self.cv_bridge.cv2_to_imgmsg(self.frame)
        # img_publish_2 = self.cv_bridge.cv2_to_imgmsg(resized_image)

        # self.modified_image_1_publish.publish(img_publish_1)
        # self.modified_image_2_publish.publish(img_publish_2)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageModification()
    rclpy.spin(node)
    rclpy.shutdown()