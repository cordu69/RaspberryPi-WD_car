#!/usr/bin/env python3

import time
import cv2
import rospy
import numpy as np
import time

import time

import numpy as np

import cv2
from PIL import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray 

def nothing():
    pass

class ColorBasedDetection():

    def __init__(self) -> None:

        # Frame rate of the camera processing.
        self.frame_rate = 36
        # Current given frame from the camera package.
        self.current_frame = None
        # Bridge between the cv2 image object the the ros image message object.
        self.cv_bridge = CvBridge()

        # Orange redish hsv color for marker
        self.hsv_min = np.array([0, 207, 38])
        self.hsv_max = np.array([10, 236, 106])

        # Image subscriber
        rospy.Subscriber("/camera/rgb_image", Image, self.get_frame_cb)

        # Object bbox publisher
        self.object_publisher = rospy.Publisher('/detection/bbox', Float32MultiArray, queue_size=1)

    def create_taskbars_and_utils(self)->None:
        """
            Create taskbars and other windows necesarry for the color checks.
        """
        #create a seperate window named 'controls' for trackbar
        cv2.namedWindow('MEASUREMENT')
        #create trackbar in 'controls' window with name 'r''
        cv2.createTrackbar('H_MIN','MEASUREMENT',0,255,nothing)
        cv2.createTrackbar('H_MAX','MEASUREMENT',0,255,nothing)
        cv2.createTrackbar('S_MIN','MEASUREMENT',0,255,nothing)
        cv2.createTrackbar('S_MAX','MEASUREMENT',0,255,nothing)
        cv2.createTrackbar('V_MIN','MEASUREMENT',0,255,nothing)
        cv2.createTrackbar('V_MAX','MEASUREMENT',0,255,nothing)

    def get_frame_cb(self, msg:Image) -> None:
        """Get the rgb frame from the camera package.

        Args:
            msg (Image): camera image message
        """
        self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detect_object(self, frame:np.array)-> np.array:
        """Detect the desired object given the current frame

        Args:
            frame (np.array): Current frame rgb raw.

        Returns:
            np.array: Frame with detected object drawn.
        """

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        white = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

        # self.hsv_min = np.array([cv2.getTrackbarPos('H_MIN', 'MEASUREMENT'), cv2.getTrackbarPos('S_MIN', 'MEASUREMENT'), cv2.getTrackbarPos('V_MIN', 'MEASUREMENT')])
        # self.hsv_max = np.array([cv2.getTrackbarPos('H_MAX', 'MEASUREMENT'), cv2.getTrackbarPos('S_MAX', 'MEASUREMENT'), cv2.getTrackbarPos('V_MAX', 'MEASUREMENT')])
        
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        zeros = np.zeros(frame.shape)
        
        # Procesing of the imnage
        result = cv2.bitwise_and(white, white, mask = mask)
        result = cv2.morphologyEx(result, cv2.MORPH_DILATE, np.ones((9,9), np.uint8), iterations = 7)
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8), iterations = 2)

        contours, hierarchy = cv2.findContours(result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        rect = None
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                
                if area > max_area:
                    correct_contour = contour
                    max_area = area
                
        if max_area > 0:
            rect = cv2.minAreaRect(correct_contour)
            bbox = cv2.boxPoints(rect)
            bbox = np.int0(bbox)
            cv2.drawContours(frame,[bbox],0,(0,0,255),2)

        return frame, rect

    def process_frame(self) -> None:
        """Process the given frame from the camera package.
        """
        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            
            if self.current_frame is not None:

                frame, rect = self.detect_object(self.current_frame)
                if rect is not None:
                    data = [rect[0][0], rect[0][1], rect[1][0], rect[1][1]]
                    float_msg = Float32MultiArray()
                    float_msg.data = data
                    self.object_publisher.publish(float_msg)

                cv2.imshow("Frame", frame)
                cv2.waitKey(1)
            
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('object_detection')
    detection = ColorBasedDetection()
    #detection.create_taskbars_and_utils()
    detection.process_frame()