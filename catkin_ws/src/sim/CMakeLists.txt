cmake_minimum_required(VERSION 3.0.2)
project(sim)

set(MSG_DEP_SET geometry_msgs std_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
    tf
	tf2
	tf2_geometry_msgs
	tf2_ros
    sbg_driver
	std_msgs
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
	rospy
    tf
	tf2
	tf2_geometry_msgs
	tf2_ros
    sbg_driver
	message_runtime
)