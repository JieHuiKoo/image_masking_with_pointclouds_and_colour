#! /usr/bin/python3
import rospy
import sys
import cv2

import numpy as np
import os

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import message_filters

import math

print("Python Version: " + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))
print("OpenCV Version: " + str(cv2.__version__))

file_num = 62

class boundingbox:
    width = float(0)
    height = float(0)
    topLeft = Point(0, 0, 0)
    bottomRight = Point(0, 0, 0)
    centroid = Point(0, 0, 0)

def imgmsg_to_cv2(img_msg):
    rgb8_flag = 0
    if img_msg.encoding != "bgr8":
        rgb8_flag = 1
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if rgb8_flag:
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def draw_bounding_boxes(input_image, boundingBoxPoints):
    
    image = input_image.copy()
    for boundingBoxLocation in boundingBoxPoints:
        image = cv2.rectangle(image, (boundingBoxLocation[0].topLeft.x, boundingBoxLocation[0].topLeft.y), (boundingBoxLocation[0].bottomRight.x, boundingBoxLocation[0].bottomRight.y), (255, 0, 0), 3)

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
  return (abs((bb1.topLeft.x + bb1.width/2) - (bb2.topLeft.x + bb2.width/2)) * 2 < (bb1.width + bb2.width)) and \
    (abs((bb1.topLeft.y + bb1.height/2) - (bb2.topLeft.y + bb2.height/2)) * 2 < (bb1.height + bb2.height))

def get_bounding_box_metrics(boundingBoxPoints):
    bb = boundingbox() 
    bb.topLeft = boundingBoxPoints[0]
    bb.bottomRight = boundingBoxPoints[1]
    bb.width = abs(bb.topLeft.x - bb.bottomRight.x)
    bb.height = abs(bb.topLeft.y - bb.bottomRight.y)
    bb.centroid = Point(bb.topLeft.x + bb.width/2, bb.topLeft.y + bb.height/2, 0)

    return bb

def get_cropped_object(image, bb_corners):

    if bb_corners:
        bb1 = bb_corners[0][0]
        bb2 = bb_corners[0][1]

        if (bb1.width * bb1.height) > (bb2.width * bb2.height):
            finalbb = bb1
        else:
            finalbb = bb2
        
        crop_img = image[finalbb.topLeft.y+3:finalbb.topLeft.y+finalbb.height-3, finalbb.topLeft.x+3:finalbb.topLeft.x+finalbb.width-3]

        return crop_img

def save_image(image, save_flag):
    global file_num
    file_directory = os.getcwd() + "/test"
    
    file_num += 1

    if not (os.path.exists(file_directory)):
        os.makedirs(file_directory)

    file_name = file_directory + "/" + str(file_num) + ".jpg"
    
    if save_flag: 
        print("Saving Image: " + file_name)
        cv2.imwrite(file_name, image)

def process_image(image_msg, bb_fromImage, bb_fromCloud):

    # Declare the cvBridge object
    proc_image = imgmsg_to_cv2(image_msg)

    bb_fromImage_corners = get_boundingBoxPoints_from_marker(bb_fromImage.points)
    bb_fromCloud_corners = get_boundingBoxPoints_from_marker(bb_fromCloud.points)

    bb_corners = get_intersecting_boundingBox(bb_fromImage_corners, bb_fromCloud_corners)

    # proc_image = draw_bounding_boxes(proc_image, bb_corners)

    cropped_object = get_cropped_object(proc_image, bb_corners)

    if cropped_object is not None:
        proc_image = cv2_to_imgmsg(cropped_object)
        
        # Define the image publisher
        image_pub = rospy.Publisher('armCamera/nearestObject_Image', Image, queue_size=100)
        image_pub.publish(proc_image)
        
        save_image(cropped_object, 0)
    else:
        pass

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

