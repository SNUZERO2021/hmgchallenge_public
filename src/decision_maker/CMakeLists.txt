cmake_minimum_required(VERSION 2.8.3)
project(decision_maker)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  hellocm_msgs
  cv_bridge
  message_filters
  tf
  tf2
  hmg_utils
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs
	DEPENDS Eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#add_library(tools src/tools.cpp)

set (SOURCES
 lib/tools.cpp
)

set (HEADERS
 include/tools.h
)

add_executable(decision_maker src/decision_maker.cpp ${HEADERS} ${SOURCES})
target_link_libraries(decision_maker ${catkin_LIBRARIES})
add_dependencies(decision_maker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#set_target_properties(decision_maker PROPERTIES COMPILE_FLAGS "-O3")

add_executable(test_dm src/test_dm.cpp ${HEADERS} ${SOURCES})
target_link_libraries(test_dm ${catkin_LIBRARIES})
add_dependencies(test_dm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
