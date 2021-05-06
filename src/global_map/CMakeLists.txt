cmake_minimum_required(VERSION 2.8.3)
project(global_map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  hmg_utils
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
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)

#add_message_files(
#  DIRECTORY msg
#  FILES 
#  Object.msg ObjectArray.msg ObjectInfo.msg Position.msg Prediction.msg PredictionArray.msg
#  Box.msg BoxArray.msg
#)

#add_service_files(
  #DIRECTORY srv
  #FILES
#  PredictionService.srv
#)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen
	opencv_highgui opencv_imgproc
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#add_library(cubic_spline_1d lib/cubic_spline_1d.cpp)
#add_library(cubic_spline_2d lib/cubic_spline_2d.cpp)
#add_library(GlobalMap lib/GlobalMap.cpp)
#add_library(History lib/History.cpp)

set (SOURCES
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/cubic_spline_1d.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/cubic_spline_2d.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/GlobalMap.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/History.cpp
)

set (HEADERS
 #${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_spline_1d.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_spline_2d.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/GlobalMap.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/History.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
)

add_executable(build_cubic_spline src/build_cubic_spline.cpp ${HEADERS} ${SOURCES})
target_link_libraries(build_cubic_spline ${catkin_LIBRARIES})
add_dependencies(build_cubic_spline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#set_target_properties(build_cubic_spline PROPERTIES COMPILE_FLAGS "-O3")

#add_executable(draw_possible_map src/draw_possible_map.cpp ${HEADERS} ${SOURCES})
#target_link_libraries(draw_possible_map ${catkin_LIBRARIES})
#add_dependencies(draw_possible_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(build_intersection_map src/build_intersection_map.cpp ${HEADERS} ${SOURCES})
target_link_libraries(build_intersection_map ${catkin_LIBRARIES})
add_dependencies(build_intersection_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_smooth src/test_smooth.cpp ${HEADERS} ${SOURCES})
target_link_libraries(test_smooth ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(test_smooth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(segment_collector src/segment_collector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(segment_collector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(segment_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})