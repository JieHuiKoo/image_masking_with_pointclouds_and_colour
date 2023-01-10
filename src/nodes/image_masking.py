#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import message_filters

import math

class boundingbox:
    width = float(0)
    height = float(0)
    topleft = Point(0, 0, 0)
    bottomright = Point(0, 0, 0)
    centroid = Point(0, 0, 0)

def draw_bounding_boxes(input_image, boundingBoxPoints):
    
    image = input_image.copy()
    for boundingBoxLocation in boundingBoxPoints:
        image = cv2.rectangle(image, (boundingBoxLocation[0].topleft.x, boundingBoxLocation[0].topleft.y), (boundingBoxLocation[0].bottomright.x, boundingBoxLocation[0].bottomright.y), (255, 0, 0), 3)

    return image

def get_boundingBoxPoints_from_marker(boundingBoxPointsMarker):
    boundingBoxPointsArray = []
    boundingBoxPointsCorners = []
    counter = 0
    for boundingBoxPoints in boundingBoxPointsMarker:
        counter += 1
        boundingBoxPointsCorners.append(Point(int(boundingBoxPoints.x), int(boundingBoxPoints.y), int(0)))
        if counter % 2 == 0:
            boundingBoxPointsArray.append(boundingBoxPointsCorners)
            boundingBoxPointsCorners = []
    
    return boundingBoxPointsArray

def get_intersecting_boundingBox(boundingBoxPoints1, boundingBoxPoints2):

    intersecting_boxes = []
    for i in range(0, len(boundingBoxPoints1)):
        for j in range(0, len(boundingBoxPoints2)):
            bb1 = get_bounding_box_metrics(boundingBoxPoints1[i])
            bb2 = get_bounding_box_metrics(boundingBoxPoints2[j])

            if do_boxes_intersect(bb1, bb2):
                intersecting_boxes.append([bb1, bb2, dist_between_bb_centroid(bb1, bb2)])

    intersecting_boxes = sorted(intersecting_boxes, key=lambda x: x[2], reverse=False)[:1]

    return intersecting_boxes

def dist_between_bb_centroid(bb1, bb2):
    return math.sqrt(pow(bb1.centroid.x - bb2.centroid.x,2) + pow(bb1.centroid.y - bb2.centroid.y, 2))

def do_boxes_intersect(bb1, bb2):
  return (abs((bb1.topleft.x + bb1.width/2) - (bb2.topleft.x + bb2.width/2)) * 2 < (bb1.width + bb2.width)) and \
    (abs((bb1.topleft.y + bb1.height/2) - (bb2.topleft.y + bb2.height/2)) * 2 < (bb1.height + bb2.height))

def get_bounding_box_metrics(boundingBoxPoints):
    bb = boundingbox() 
    bb.topleft = boundingBoxPoints[0]
    bb.bottomright = boundingBoxPoints[1]
    bb.width = abs(bb.topleft.x - bb.bottomright.x)
    bb.height = abs(bb.topleft.y - bb.bottomright.y)
    bb.centroid = Point(bb.topleft.x + bb.width/2, bb.topleft.y + bb.height/2, 0)

    return bb

def process_image(image_msg, bb_fromImage, bb_fromCloud):

    # Declare the cvBridge object
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(image_msg, "bgr8")

    bb_fromImage_corners = get_boundingBoxPoints_from_marker(bb_fromImage.points)
    bb_fromCloud_corners = get_boundingBoxPoints_from_marker(bb_fromCloud.points)

    bb_corners = get_intersecting_boundingBox(bb_fromImage_corners, bb_fromCloud_corners)

    resized = draw_bounding_boxes(orig, bb_corners)

    image_message = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
    
    image_pub = rospy.Publisher('armCamera/nearestObject', Image, queue_size=100)

    image_pub.publish(image_message)

def start_node():
    rospy.init_node('image_masking')
    rospy.loginfo('image_masking node started')

    image_sub = message_filters.Subscriber("armCamera/segmentedBlobs_RawImage", Image)
    boundingBoxPoints_fromCloud_sub = message_filters.Subscriber("/armCamera/nearestCloudCluster_BoundingBoxPoints", Marker)
    boundingBoxPoints_fromImage_sub = message_filters.Subscriber("/armCamera/segmentedBlobs_BoundingBoxPoints", Marker)

    # Subscribe to the 3 topics
    # Slop refers to delay we can wait for message in seconds
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, boundingBoxPoints_fromImage_sub, boundingBoxPoints_fromCloud_sub], queue_size=1, slop=1000000000000000000, allow_headerless=False)
    ts.registerCallback(process_image)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass