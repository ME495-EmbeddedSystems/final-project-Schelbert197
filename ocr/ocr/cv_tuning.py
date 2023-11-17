import cv2
import imutils
from imutils.perspective import four_point_transform
from imutils import contours

def nothing(x):
    pass

class CV():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        cv2.namedWindow('Parameters')
        cv2.createTrackbar('Canny_T_min', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar('Canny_T_max', 'Parameters', 0, 255, nothing)
        cv2.setTrackbarPos('Canny_T_max', 'Parameters', 255)


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
            blurred = cv2.GaussianBlur(gray, (11, 11), 0)
            cv2.imshow("blurred", blurred)

            ret3,th3 = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            cv2.imshow("binarized", th3)

            c_min = cv2.getTrackbarPos('Canny_T_min', 'Parameters')
            c_max = cv2.getTrackbarPos('Canny_T_max', 'Parameters')
            edged = cv2.Canny(blurred, c_min, c_max)
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

            try:
                return warped
            except:
                return 0

def main():
    cv = CV()
    while True:
        cv.image_modification()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()