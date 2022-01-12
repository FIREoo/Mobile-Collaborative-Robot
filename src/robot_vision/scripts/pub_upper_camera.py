#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys

# print(cv2.getBuildInformation())

rospy.init_node('camera_node', anonymous=True)
pub = rospy.Publisher('camera/image', Image, queue_size=1)
rate = rospy.Rate(10)  # 10hz

#v4l2-ctl --list-devices
#AVerMedia PW313D (L) 1920*1080 (6)
index = rospy.get_param('~device_index')
cap = cv2.VideoCapture(int(index), cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

width = rospy.get_param('~image_width')
height = rospy.get_param('~image_height')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
# cap.set(6, codec)
# cap.set(5, 30)
# width = rospy.get_param('~image_width')
# cap.set(3, int(width))
# cap.set(cv2.cap_width, int(width))
# height = rospy.get_param('~image_height')
# cap.set(4, int(height))

count = 0
while not cap.isOpened():
    rospy.logwarn("Cannot open camera :", index)
    rospy.sleep(1)
    count += 1
    if (count == 3):
        exit()

bridge = CvBridge()

fps = cap.get(cv2.CAP_PROP_FPS)
info = ("\033[0;42m" + str(cap.getBackendName()) + " Capture:" + str(index) + "  [" + str(width) + "x" + str(height) + "](" + str(fps) + ")\033[0m")
rospy.loginfo(info)

while not rospy.is_shutdown():

    ret, cv_image = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        # break

    else:
        # cv2.imshow("img", cv_image)
        try:
            #Converting OpenCV images to ROS image messages
            image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print('bridge error', e)
        pub.publish(image_message)
        cv2.waitKey(1)
        rate.sleep()