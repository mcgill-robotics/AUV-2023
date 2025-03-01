cmake_minimum_required(VERSION 3.14.4)
project(bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Reference one-to-one convertion guide for how this was converted from the original
# https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-CPP-Packages.html
find_package(ament_cmake_ros REQUIRED)

find_package(state_estimation REQUIRED)

ament_export_dependencies(
	controls
	planner
	propulsion
	state_estimation
)

ament_package()	