cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	geometry_msgs
	geometric_shapes
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_benchmarks
	moveit_task_constructor_msgs
	moveit_visual_tools
	roscpp
	visualization_msgs
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		moveit_ros_benchmarks
		moveit_task_constructor_msgs
		moveit_visual_tools
		visualization_msgs
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

