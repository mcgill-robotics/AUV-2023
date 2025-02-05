cmake_minimum_required(VERSION 3.8)
project(auv_msgs)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set of packages whose message types are used
# actionlib_msgs are now included in ROS2 and should be removed?
set(MSG_DEP_SET 
	geometry_msgs 
	std_msgs
#	actionlib_msgs
)

set(msg_files	
	"msg/DeadReckonReport.msg"
	"msg/PingerBearing.msg"
	"msg/PingerTimeDifference.msg"
	"msg/ThrusterForces.msg"
	"msg/ThrusterMicroseconds.msg"
	"msg/UnityState.msg"
	"msg/VelocityReport.msg"
	"msg/VisionObject.msg"
	"msg/VisionObjectArray.msg"
)

set(action_files
	"action/Effort.action"
	"action/StateQuaternion.action"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dep IN LISTS MSG_DEP_SET)
	find_package(${dep} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${action_files}
	DEPENDENCIES ${MSG_DEP_SET}
)

ament_export_dependencies(${MSG_DEP_SET})

ament_package()




