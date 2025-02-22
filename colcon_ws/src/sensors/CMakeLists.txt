cmake_minimum_required(VERSION 3.14.4)
project(sensors)

# Set CMAKE_CXX_STANDARD 17 (required for ament_cmake)
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MSG_DEP_SET 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  auv_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(auv_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
# Alternatives fro the following packages must be implemented as well after speaking to electrical
# sbg_driver
# rosserial_arduino
# rosserial_client

# find message dependencies
foreach(dep IN LISTS MSG_DEP_SET)
	find_package(${dep} REQUIRED)
endforeach()

# Commented because hypothesis that only message declarations require invoking the idl
# rosidl_generate_interfaces(${PROJECT_NAME}
# 	DEPENDENCIES ${MSG_DEP_SET}
# )

# Add executables and link to their dependencies
add_executable(depth_republisher
  src/depth_republish.cpp
)
add_executable(imu_republisher
  src/imu_republish.cpp
)
add_executable(dvl_republisher
  src/dvl_republish.cpp
)

# Replaces include_directories(${catkin_INCLUDE_DIRS}) in ROS1
target_include_directories(depth_republisher
  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(imu_republisher
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(dvl_republisher
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

find_package(rclcpp REQUIRED)
# Links nodes to rclcpp and geometry_msgs required libraries 
ament_target_dependencies(depth_republisher
  rclcpp
  std_msgs
  geometry_msgs
)
ament_target_dependencies(imu_republisher
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
ament_target_dependencies(dvl_republisher
  rclcpp
  std_msgs
  geometry_msgs
)

# Install executable and scripts
install(TARGETS
  depth_republisher
  dvl_republisher
  imu_republisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install your shell scripts or other executable programs
# Redundant as bridge does not work.
# install(PROGRAMS
#   src/start_zed_bridge.sh
#   DESTINATION ${AMENT_PACKAGE_BIN_DESTINATION}
# )

# Install python scripts
install(PROGRAMS 
  ${PROJECT_NAME}/calibrate_dvl.py
  ${PROJECT_NAME}/covariance.py
  ${PROJECT_NAME}/dvl_covariance.py
  ${PROJECT_NAME}/imu_covariance.py
  ${PROJECT_NAME}/calibrate_dvl.py
  ${PROJECT_NAME}/print_hydrophones.py
  ${PROJECT_NAME}/sensors_status.py
  ${PROJECT_NAME}/tether_status.py
  ${PROJECT_NAME}/waterlinked_dvl.py
  DESTINATION lib/${PROJECT_NAME}
)

# Declare package's runtime dependencies
find_package(ament_cmake REQUIRED)
ament_export_dependencies(${MSG_DEP_SET})
ament_package()

# Checks if ROS2 and licensing conventions are followed, can be omitted.
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
