import rclpy
from rclpy.node import Node
import numpy as np
import imutils
from imutils.perspective import four_point_transform

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
        cv2.imshow("binarised", binarised)
        c_min = cv2.getTrackbarPos('Canny_T_min', 'Parameters')
        c_max = cv2.getTrackbarPos('Canny_T_max', 'Parameters')
        edged = cv2.Canny(blurred, c_min, c_max)
        cv2.imshow("edged", edged)
        # edged_2 = cv2.Canny(gray, c_min, c_max)
        # cv2.imshow("edged_2", edged_2)

        # find contours in the edge map, then sort them by their
        # size in descending order
        cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,
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