#!/usr/bin/env python3

# Python Libs
import sys
import time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage

class LaneDetector:
    def _init_(self):
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/myduckie/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        rospy.init_node("my_lane_detector")
        
    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8") 
       
        # Apply color space conversion directly to the cropped image
        hsv_img = cv2.cvtColor(img[100:500, 100:600], cv2.COLOR_BGR2HSV)   
        # Create masks for white and yellow colors using cv2.inRange()
        white_pixel = cv2.inRange(hsv_img, np.array([0, 0, 200])  ,np.array([255, 50, 255]) )
        yellow_pixel = cv2.inRange(hsv_img, np.array([20, 100, 100]), np.array([40, 255, 255]))

        line_white = self.Hough_lines_apply(cv2.Canny(white_pixel, 50, 150))
        line_yellow = self.Hough_lines_apply(cv2.Canny(yellow_pixel, 50, 150))

        self.detect_line_draw(crop_section, line_white)
        self.detect_line_draw(crop_section, line_yellow)

        cv2.imshow('White Filter', white_pixel)
        cv2.imshow('Yellow Filter', yellow_pixel)
        cv2.imshow('Detected Lines', crop_section)
        cv2.waitKey(1)

    def Hough_lines_apply(self, img):
        return cv2.HoughLinesP(img, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=50)
       

    def detect_line_draw(self, img, lines):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    def run(self):
        rospy.spin()

if _name_ == "_main_":
    try:
        lane_detector_instance = LaneDetector()
        lane_detector_instance.run()
        
    except rospy.ROSInterruptException:
        pass
