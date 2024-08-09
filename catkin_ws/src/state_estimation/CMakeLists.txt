cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

set(MSG_DEP_SET geometry_msgs std_msgs auv_msgs nav_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
  sensors
  roscpp REQUIRED
	rospy
  tf
	tf2
	tf2_geometry_msgs
	tf2_ros
  tf2_msgs
)

catkin_package( CATKIN_DEPENDS
	${MSG_DEP_SET}
  sensors
  roscpp
	rospy
	tf2
	tf2_geometry_msgs
	tf2_ros
	INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)



add_executable(odom_republisher
  src/odom_republisher.cpp
)

target_link_libraries(
  odom_republisher
  ${roscpp_LIBRARIES} 
  ${tf2_ros_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)



install(
  TARGETS odom_republisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/odom_republisher
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)