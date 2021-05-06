cmake_minimum_required(VERSION 3.0.2)
project(vehicle_info)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  hmg_utils
  tf
  tf2
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs

	DEPENDS
	opencv_highgui opencv_imgproc
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

set (SOURCES 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/example.cpp
)

set (HEADERS
# ${CMAKE_CURRENT_SOURCE_DIR}/include/example.h
)

add_executable(kalman_filter src/kalman_filter.cpp ${HEADERS} ${SOURCES})
add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter ${catkin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
#set_target_properties(kalman_filter PROPERTIES COMPILE_FLAGS "-O3")

add_executable(control_data_collector src/control_data_collector.cpp ${HEADERS} ${SOURCES})
add_dependencies(control_data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_data_collector ${catkin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
#set_target_properties(kalman_filter PROPERTIES COMPILE_FLAGS "-O3")

