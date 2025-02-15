cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
	controls	
	planner
	propulsion
	state_estimation
)

catkin_package(	CATKIN_DEPENDS 
	controls	
	planner
	propulsion
	state_estimation
)

