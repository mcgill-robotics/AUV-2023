# Controls

## Overview


The controls package is responsible for determining the effort the AUV should exert at a given moment in time based on a target and the current state.

## Servers

The controls takes into from two different servers representing two different modes of operation. 

### State Server

The state server accepts a pose (position + orientation), and publishes to the PIDs to enter the desired state. It monitors the pose and waits for the auv to settle in the correct pose before completing. Preempting this server causes to publish the current pose to the PIDs, effectively braking the auv. When sending a goal to the state server, you are allowed to specify which axes to consider and which to ignore. For example you can tell it to descend by some amount without affecting the motion in the other 5 axes. You are also allowed to tell the state server to interpret the goal as a displacement.

### Effort Server

The effort server accepts a combination of surge, sway, heave, roll, pitch, and yaw values. These values should be interpreted as forces which will be sent to superimposer to be combined into an effort. When sending a goal to the server, you are allowed to specify which axes to consider and which to ignore. For example, you can tell it to surge by some amount which modifying the values being published to sway,heave ... etc.

### License

The source code is released under a GPLv3 license.


### Published Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `effort` | `geometry_msgs/Wrench` | Force and torque (N and N\*m), relative to the robot's frame of reference to be applied at a given moment |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `state` | `geometry_msgs/Pose` | The current estimate of the position/orientation (pose) of the AUV |
| `state_x` | `std_msgs/Float64` | X position|
| `state_y` | `std_msgs/Float64` | Y position|
| `state_z` | `std_msgs/Float64` | Z position|
| `state_theta_x` | `std_msgs/Float64` | Rotation about the x axis in euler angles|
| `state_theta_y` | `std_msgs/Float64` | Rotation about the y axis in euler angles|
| `state_theta_z` | `std_msgs/Float64` | Rotation about the z axis in euler angles|



### Actions

This package implements a Waypoint action server as specified in auv_msgs.

| Action | description |
| ------ | ------|
| `StateAction` | A desired pose for the auv to enter. |
| `EffortAction` | A desired surge/sway/heave/roll/pitch/yaw combination. |


### Dependencies

- `auv_msgs`
- `catkin`
- `geometry_msgs`
- `pid`
- `rospy`
- `std_msgs`
- `tf`

### Running

Launch all package nodes

	roslaunch controls controls.launch
