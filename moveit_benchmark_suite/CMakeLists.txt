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

add_library(${PROJECT_NAME}
  src/benchmark_parameters.cpp)

add_executable(${PROJECT_NAME}_load_benchmark src/load_benchmark.cpp)
target_link_libraries(${PROJECT_NAME}_load_benchmark ${PROJECT_NAME} ${catkin_LIBRARIES})
