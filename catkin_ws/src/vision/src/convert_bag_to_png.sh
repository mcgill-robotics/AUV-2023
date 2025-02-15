#!/bin/bash

# Prompt the user to enter the image topic name
echo "Enter the image topic name (e.g., /vision/front_cam/image_rgb):"
read image_topic

# Prompt the user to enter the seconds per frame
echo "Enter the seconds per frame (e.g., 0.01). You can find this by running rosbag info on the bag file and dividing the duration (in seconds) by the number of frames."
read sec_per_frame

echo "Enter the image encoding (default is bgr8)."
read encoding

echo "Running image converter. Play the bag file in a seperate terminal instance."

rosrun image_view image_saver _encoding:=$encoding _sec_per_frame:=$sec_per_frame image:=$image_topic
