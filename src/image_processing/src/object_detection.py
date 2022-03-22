#!/usr/bin/env python3

from picamera.array import PiRGBArray
from picamera import PiCamera


import time
import cv2
import rospy
import numpy as np
import time


class ObjectDetection:

    def __init__(self, configPath = "/home/pi/catkin_ws/src/image_processing/rsc/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt",
                        weightsPath = "/home/pi/catkin_ws/src/image_processing/rsc/frozen_inference_graph.pb"):
        self.current_available_classes = ['cat']

        # Camera settings
        self.camera = PiCamera()
        self.camera.resolution = (370,400)
        self.camera.framerate = 30
        self.camera.brightness = 65
        self.camera_capture = PiRGBArray(self.camera)

        self.classNames = []
        classFile = "/home/pi/catkin_ws/src/image_processing/rsc/coco.names"
        with open(classFile,"rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

        self.net = cv2.dnn_DetectionModel(weightsPath,configPath)
        self.net.setInputSize(320,320)
        self.net.setInputScale(1.0/ 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)


    def detect_objects(self, frame, thres=0.5, nms=0.2, draw=True, objects=[]):
        classIds, confs, bbox = self.net.detect(frame,confThreshold=thres,nmsThreshold=nms)
        
        if len(objects) == 0: 
            objects = self.classNames
            objectInfo=[]
        if len(classIds) != 0:
            for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
                className = self.classNames[classId - 1]
                if className in objects:
                    objectInfo.append([box,className])
                    if (draw):
                        cv2.rectangle(frame,box,color=(0,255,0),thickness=2)
                        cv2.putText(frame,self.classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                        cv2.putText(frame,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)

        return frame,objectInfo

    def compute_detection(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            
            for frame in self.camera.capture_continuous(self.camera_capture, format='bgr', use_video_port=True):

                frame = frame.array
                start = time.time()
                frame, detect_objects = self.detect_objects(frame)
                print(time.time() - start)
                cv2.imshow('frame', frame)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break

                self.camera_capture.truncate(0)
                #rate.sleep()


if __name__ == "__main__":
    rospy.init_node('object_detection')
    detector = ObjectDetection()
    detector.compute_detection()