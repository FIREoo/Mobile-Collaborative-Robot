#!/usr/bin/env python3
import cv2
import rospy
import cv2
import os
import numpy as np
from rospy.core import rospywarn
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from robot_vision.msg import YoloBoundingBox


def img_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr('bridge error {e}')
    img_detect = cv2.resize(cv_image, (detect_width, detect_height), interpolation=cv2.INTER_AREA)
    # img_detect = cv2.resize(cv_image, (960, 540), interpolation=cv2.INTER_AREA)
    classIds, scores, boxes = model.detect(img_detect, confThreshold=0.6, nmsThreshold=0.4)

    for (classId, score, box) in zip(classIds, scores, boxes):
        cv2.rectangle(img_detect, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(63, 192, 209), thickness=2)
        text = "{}({:0.2f})".format(str(classes[int(classId)]), float(score))
        cv2.putText(img_detect, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(63, 192, 209), thickness=1)
        # cv2.putText(img_detect, str(classes[int(classId)]), (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(63, 192, 209), thickness=1)
        if (int(classId) == 0):  #hand
            msg_box = YoloBoundingBox()
            msg_box.object_name = 'Hand'
            msg_box.x = int(box[0] / detect_width * origin_width)
            msg_box.y = int(box[1] / detect_height * origin_height)
            msg_box.width = int(box[2] / detect_width * origin_width)
            msg_box.height = int(box[3] / detect_height * origin_height)
            pub_box.publish(msg_box)

    try:
        #Converting OpenCV images to ROS image messages
        image_message = bridge.cv2_to_imgmsg(img_detect, "bgr8")
    except CvBridgeError as e:
        print('bridge error', e)
    pub.publish(image_message)

    # cv2.imshow('Image', img_detect)
    # cv2.waitKey(2)


#__Main__

rospy.init_node('yolo_node', anonymous=True)

bridge = CvBridge()
path_of_data = os.path.abspath('../../../data/')
path_of_data += '/'

# rospy.loginfo('path of data : {path_of_data}')
rospy.loginfo('path of data :' + path_of_data)

file_names = rospy.get_param('~yolo_names')
with open(path_of_data + file_names, 'r') as f:
    classes = f.read().splitlines()

#GPU settings
rospy.loginfo('setting preferable backend and target to CUDA')
file_cfg = rospy.get_param('~yolo_cfg')
file_weights = rospy.get_param('~yolo_weights')
net = cv2.dnn.readNetFromDarknet(path_of_data + file_cfg, path_of_data + file_weights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
model = cv2.dnn_DetectionModel(net)

origin_width = 2048
origin_height = 1536
detect_width = 512
detect_height = 384
# model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)
model.setInputParams(scale=1 / 255, size=(detect_width, detect_height), swapRB=True)

sub = rospy.Subscriber('/camera/image', Image, img_callback, queue_size=1)
pub = rospy.Publisher(sub.name + '/yolo', Image, queue_size=1)
pub_box = rospy.Publisher('yolo/hand/box', YoloBoundingBox, queue_size=1)

rospy.spin()
# cv2.destroyAllWindows()
