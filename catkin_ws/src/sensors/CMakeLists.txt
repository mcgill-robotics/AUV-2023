cmake_minimum_required(VERSION 3.0.2)
project(sensors)

set(MSG_DEP_SET std_msgs sensor_msgs auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  rospy
  roscpp
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
)

catkin_package( CATKIN_DEPENDS
  ${MSG_DEP_SET}
  roscpp
  rospy
  geometry_msgs
  sbg_driver
  rosserial_arduino
  rosserial_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_executable(depth_republisher
  src/depth_republish.cpp
)

add_executable(imu_republisher
  src/imu_republish.cpp
)

target_link_libraries(
  depth_republisher
  ${roscpp_LIBRARIES} 
  ${geometry_msgs_LIBRARIES}
)

target_link_libraries(
  imu_republisher
  ${roscpp_LIBRARIES} 
  ${geometry_msgs_LIBRARIES}
)

install(PROGRAMS
  src/start_zed_bridge.sh
  scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)