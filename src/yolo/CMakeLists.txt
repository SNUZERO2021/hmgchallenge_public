cmake_minimum_required(VERSION 2.8.3)
project(yolo)

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
  geometry_msgs
  sensor_msgs
  message_generation
  cv_bridge
  message_filters
  tf
  tf2
  hmg_utils
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
find_package(PCL 1.8 REQUIRED)


generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen
	opencv_highgui opencv_imgproc
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/yolo.py scripts/darknet.py scripts/darknet_video.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
