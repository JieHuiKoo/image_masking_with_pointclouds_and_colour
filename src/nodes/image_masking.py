#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def process_image(msg):
    #Resize params
    resize_x = 1
    resize_y = 1

    # Declare the cvBridge object
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    drawImg = orig
    
    # Resize the image 
    resized = cv2.resize(orig, None, fx=resize_x, fy=resize_y)
    drawImg = resized

    image_message = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
    
    image_pub = rospy.Publisher('armCamera/nearestObject', Image, queue_size=10)
    image_pub.publish(image_message)

def start_node():
    rospy.init_node('image_masking')
    rospy.loginfo('image_masking node started')

    rospy.Subscriber("/armCamera/color/image_raw", Image, process_image)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass