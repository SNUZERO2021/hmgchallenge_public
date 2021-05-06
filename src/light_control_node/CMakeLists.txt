cmake_minimum_required(VERSION 2.8.3)
project(light_control_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rosbag
  std_msgs
  cmrosutils
  hellocm
  hellocm_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters
  hmg_utils
  hellocm_msgs
)

find_package(Eigen3)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(light_control_node src/light_control_node.cpp)
target_link_libraries(light_control_node ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(light_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})