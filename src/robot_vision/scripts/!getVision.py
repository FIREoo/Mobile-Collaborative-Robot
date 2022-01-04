#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print('bridge error', e)

    cv_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("Image window", cv_hsv)
    cv2.waitKey(3)


bridge = CvBridge()
rospy.init_node('UI_node', anonymous=True)
sub = rospy.Subscriber('camera/image', Image, callback)
rospy.spin()
