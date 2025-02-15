cmake_minimum_required(VERSION 3.0.2)
project(vision)

set(MSG_DEP_SET sensor_msgs auv_msgs) 

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
)

catkin_package(CATKIN_DEPENDS 
	${MSG_DEP_SET}
	rospy
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/launch/test_lane_marker_measure.test)
endif()
