#!/usr/bin/env python3

import time
import cv2
import rospy

import numpy as np

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera():

    def __init__(self) -> None:

        # Frame rate of the camera processing
        self.frame_rate = 36
        # Width of the processed image
        self.width = 640
        # Height of the processed image
        self.height = 480

        # Apply the settings to the camera cap
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width) # width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height) # height
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate) # fps

        # Bridge to transform cv2 image to sensor msg image
        self.cv_bridge = CvBridge()

        # Publisher
        self.rgb_publisher = rospy.Publisher("/camera/rgb_image", Image, queue_size=1)

    def get_frame(self):
        while not rospy.is_shutdown():
            ret, image = self.cap.read()
            if not ret:
                raise RuntimeError("failed to read frame")
            image_message = self.cv_bridge.cv2_to_imgmsg(np.array(image), "bgr8")

            self.rgb_publisher.publish(image_message)

if __name__ == "__main__":
    rospy.init_node('camera')
    camera = Camera()
    camera.get_frame()