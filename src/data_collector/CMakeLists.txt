cmake_minimum_required(VERSION 2.8.3)
project(data_collector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  cv_bridge
  message_filters
  tf
  tf2
  hmg_utils
#  OpenCV
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
find_package(PCL 1.8 REQUIRED)

#add_message_files(
#  DIRECTORY msg
#  FILES 
#)

#add_service_files(
#  DIRECTORY srv
#  FILES
#)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen opencv_highgui opencv_imgproc
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_executable(data_collector src/data_collector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(data_collector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(data_visualizer src/data_visualizer.cpp ${HEADERS} ${SOURCES})
target_link_libraries(data_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(data_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_collector src/control_collector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(control_collector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(control_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(image_collector src/image_collector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(image_collector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(image_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
