# Vision


## Overview


The vision package is responsible for using the cameras on the AUV to help navigate the AUV relative to various objects in the pool.

For training YOLO models on custom data, see /cv-model folder.

The vision package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Published Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/vision/viewframe_detection` | `auv_msgs/ObjectDetectionFrame` | Bounding box, confidence, class id, and camera on which detection was made of all objects in the viewframe of the AUV |
| `/vision/down_visual` | `sensor_msgs/Image` | Visualization of all detections on the downwards camera of the AUV |
| `/vision/lane_marker_threshold` | `sensor_msgs/Image` | Visualization of thresholding to identify pixels that belong to a lane marker on the downwards camera input image |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/vision/down_cam/image_raw` | `sensor_msgs/Image` | Images taken by the downwards camera |
| `/vision/down_visual` | `sensor_msgs/Image` | Visualization of all detections on the downwards camera of the AUV |
| `/vision/lane_marker_threshold` | `sensor_msgs/Image` | Visualization of thresholding to identify pixels that belong to a lane marker on the downwards camera input image |

## Installation

### Dependencies

- `catkin`
- `auv_msgs`
- `sensor_msgs`
- `ultralytics`
- `pickle5`
- `opencv-python`
- `ros-noetic-cv-bridge`
- `ros-noetic-usb-cam`
- `ros-noetic-image-view`
- `ros-noetic-rqt-gui`
- `ros-noetic-rqt-gui-image-view`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2025>/catkin_ws/src
	catkin build vision

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Note: usb_cam will display warnings and errors to the screen, this is normal.

Launch all package nodes

	roslaunch vision vision.launch

Launch object detection node and rqt-gui with vision perspective loaded

	roslaunch vision object_detection.launch

Launch feed recording node (records feeds to file system as video files in one minute increments)

	roslaunch vision record_feeds.launch
	
