from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, frame):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,50,70])
        upper_red1 = np.array([10,255,255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([160,50,50])
        upper_red2 = np.array([180,255,255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2
        res = cv2.bitwise_and(frame,frame, mask= mask)
        img = cv2.medianBlur(res, 5)

        cimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 60,                  
                         param1=30, param2=10, minRadius=0, maxRadius=0)
        if circles is not None:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
