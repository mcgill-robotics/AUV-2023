# Propulsion


## Overview


The propulsion package is responsible for moving the AUV and provides a hardware-agnostic interface.

The propulsion package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Published Topics

*none*

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/effort` | `geometry_msgs/Wrench` | Force and torque, relative to the robot's frame of reference to be applied at a given moment |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `propulsion_msgs`

### Building

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build propulsion

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Flash arduino

	catkin build --no-deps  propulsion --make-args propulsion_embedded_thrusters-upload

Launch all package nodes

	roslaunch propulsion propulsion.launch
	
### Usage

Publishing a `geometry_msgs/Wrench` message onto `/effort` topic:

	
	rostopic pub -1 /effort geometry_msgs/Wrench "{force: {x: 1.0, y: 0.0, z: -0.5}, torque: {x: 1.0, y: -0.5, z: -2.0}}"
