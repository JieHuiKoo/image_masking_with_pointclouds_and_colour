#! /home/jiehui/anaconda3/envs/tensorflow/bin/python
import rospy
import sys

import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2

from collections import Counter

from image_masking_with_pointclouds_and_colour.srv import GetPickupCentroid,GetPickupCentroidResponse

print("Python Version: " + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))

class object_properties:
    class_name = String('')
    location = PoseStamped()
    class_buffer = []
    buffer_len = 20
    pointcloud = PointCloud2()
    timestamp = rospy.Time()
    valid = False

    def add_to_class_buffer(self, class_name):
        if len(self.class_buffer) >= self.buffer_len:
            self.class_buffer.pop(0)
        self.class_buffer.append(class_name.data)


    def get_object_class(self):
        occurance_count = Counter(self.class_buffer)
        
        if len(self.class_buffer) < self.buffer_len:
            return "Not_Ready_Yet"
        else:
            return occurance_count.most_common(1)[0][0]

observed_object = object_properties()

def convert_string_to_rosMsg(python_string):
    ros_msg = String()
    ros_msg.data = python_string
    return ros_msg

def record_object_params(object_class_msg, object_poseStamped_msg, object_is_bb_intersecting_msg, object_pointcloud_msg):
    
    if object_is_bb_intersecting_msg:
        if (object_pointcloud_msg.height * object_pointcloud_msg.width) > 0:
            observed_object.add_to_class_buffer(object_class_msg)
            observed_object.class_name = observed_object.get_object_class()
            observed_object.location = object_poseStamped_msg
            observed_object.pointcloud = object_pointcloud_msg
            observed_object.timestamp = object_pointcloud_msg.header.stamp
    
            if observed_object.class_name != "Not_Ready_Yet":
                print("[GetPickupCentroid]: Object Valid")
                observed_object.valid = True
                return
    
    print("[GetPickupCentroid]: Object Invalid")
    observed_object.valid = False
    
def is_within_reasonable_delay(observed_object):
    return ((rospy.Time.now() - observed_object.timestamp).to_sec() < 1000)

def handle_get_pickup_location(req):
    rospy.loginfo("[GetPickupCentroid]: Request received")
    rospy.loginfo("[GetPickupCentroid]: Object Class: %s", observed_object.get_object_class())

    return (GetPickupCentroidResponse(\
        object_centroid=observed_object.location, \
        object_class=convert_string_to_rosMsg(observed_object.get_object_class()), \
        object_pointcloud=observed_object.pointcloud, \
        object_valid=observed_object.valid))

def start_node():
    rospy.init_node('get_pickup_centroid')
    rospy.loginfo('[GetPickupCentroid]: Service started')
    
    object_class = message_filters.Subscriber("armCamera/nearestObject_Class", String)
    object_poseStamped = message_filters.Subscriber("armCamera/nearestCloudCluster_Centroid", PoseStamped)
    object_is_bb_intersecting = message_filters.Subscriber("armCamera/nearestObject_Detected", Bool)
    object_pointcloud = message_filters.Subscriber("armCamera/nearestCloudCluster", PointCloud2)

    # Subscribe to the 3 topics
    # Slop refers to delay we can wait for message in seconds
    ts = message_filters.ApproximateTimeSynchronizer([object_class, object_poseStamped, object_is_bb_intersecting, object_pointcloud], queue_size=1, slop=1000000000000, allow_headerless=True)
    ts.registerCallback(record_object_params)
    
    s = rospy.Service('GetPickupCentroid', GetPickupCentroid, handle_get_pickup_location)    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()

    except rospy.ROSInterruptException:
        pass