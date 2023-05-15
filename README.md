# image_masking_with_pointclouds_and_colour

About: This package takes in an RGB image as well as the coordinates of the bounding boxes produced by the following packages:
object_segmentation_with_pointclouds
object_segmentation_with_colour

A ROS service is also implemented to interface with the information available.

To run:

Execute the following commands
```
rosrun image_masking_with_pointclouds_and_colour image_masking.py # ON robot
rosrun image_masking_with_pointclouds_and_colour get_pickup_centroid.py # ON PC
```
