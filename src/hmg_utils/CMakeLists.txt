cmake_minimum_required(VERSION 2.8.3)
project(hmg_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  hellocm_msgs
  message_filters
  tf
  tf2
)

find_package(Eigen3)
find_package(PCL 1.8 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES 
  Target.msg
  Targets.msg
)

add_message_files(
  DIRECTORY msg
  FILES 
  Object.msg ObjectArray.msg ObjectInfo.msg Position.msg Prediction.msg PredictionArray.msg
  Box.msg BoxArray.msg ConvexHull.msg ConvexHullArray.msg Pedestrian.msg PedestrianArray.msg
  Light.msg PredictionTrackerTime.msg ObjectId.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  NearestSegment.srv
  SegmentArray.srv
  PredictionService.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
  	${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  lib/cubic_spline_1d.cpp
  lib/cubic_spline_2d.cpp
  lib/GlobalMap.cpp
  lib/Lidar.cpp
  lib/ObjectSensor.cpp
  lib/utils.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
