cmake_minimum_required(VERSION 3.0.2)
project(propulsion)

# set of packages whose message types are used
set(MSG_DEP_SET auv_msgs geometry_msgs) 

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	roscpp
	rospy
)

catkin_package(CATKIN_DEPENDS 
	${MSG_DEP_SET}
	roscpp
	rospy
)

