#! /usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import numpy as np

bridge = CvBridge()
IMAGE_TOPIC_NAME = "/camera/color/image_raw"
DEPTH_IMAGE_TOPIC_NAME = "/camera/aligned_depth_to_color/image_raw"
cv_image = None
red_image = None

rospy.init_node("image_finder")
pub = rospy.Publisher('/red_image', Image, queue_size=10)
info_pub = rospy.Publisher('/info', CameraInfo, queue_size=10)
box = None
mask = None
red_mask = None
info = None

def callback(data):
    global cv_image, red_image, red_mask
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower = np.array([130, 13, 30], np.uint8)
    upper = np.array([192, 30, 90], np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    red_image = output
    red_image = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)

    contours = cv2.findContours(red_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    result = cv_image.copy()
    
    rect_mask = np.zeros(cv_image.shape[:2], dtype="uint8")    

    for cntr in contours:
        x,y,w,h = cv2.boundingRect(cntr)
        if w > 50 and h > 50:
            cv2.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
            box = (x, y, w, h)
            cv2.rectangle(rect_mask, (x, y), (x+w, y+h), (255, 255, 255), -1)
            red_mask = rect_mask.copy()
            break
        else:
            red_mask = np.zeros(cv_image.shape[:2], dtype="uint8")

def info_callback(data):
    global info
    info = data
    

def depth_callback(data):
    if red_mask is None:
        return
    
    if info is None:
        return
    
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    depth_filtered = cv2.bitwise_and(depth_image, depth_image, mask=red_mask) 

    red_msg = bridge.cv2_to_imgmsg(depth_filtered, encoding="passthrough")
    pub.publish(red_msg)
    info_pub.publish(info)

rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, info_callback)
rospy.Subscriber(IMAGE_TOPIC_NAME, Image, callback)
rospy.Subscriber(DEPTH_IMAGE_TOPIC_NAME, Image, depth_callback)

rospy.spin()
