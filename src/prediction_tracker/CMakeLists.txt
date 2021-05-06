cmake_minimum_required(VERSION 2.8.3)
project(prediction_tracker)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

##### CARMAKER #####
#set(CARMAKER_VER 9.0.2)
#set(CARMAKER_NUMVER 90002)

#set(CARMAKER_DIR $ENV{IPGHOME}/carmaker/linux64-${CARMAKER_VER})
#set(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)
#set(CMAKE_POSITION_INDEPENDENT_CODE_ON)
####################

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

#add_message_files(
#  DIRECTORY msg
#  FILES 
#  Object.msg ObjectArray.msg ObjectInfo.msg Position.msg Prediction.msg PredictionArray.msg
#  Box.msg BoxArray.msg
#)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen
	opencv_highgui opencv_imgproc
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
# ${CARMAKER_INC_DIR}
 )

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

add_executable(obstacle_detector src/obstacle_detector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(prediction_tracker src/prediction_tracker.cpp ${HEADERS} ${SOURCES})
target_link_libraries(prediction_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(prediction_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#set_target_properties(prediction_tracker PROPERTIES COMPILE_FLAGS "-O3")

add_executable(prediction_visualizer src/prediction_visualizer.cpp ${HEADERS} ${SOURCES})
target_link_libraries(prediction_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(prediction_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(prediction_evaluation src/prediction_evaluation.cpp ${HEADERS} ${SOURCES})
target_link_libraries(prediction_evaluation ${catkin_LIBRARIES})
add_dependencies(prediction_evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(segment_service src/segment_service.cpp ${HEADERS} ${SOURCES})
target_link_libraries(segment_service ${catkin_LIBRARIES})
add_dependencies(segment_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} prediction_tracker_gencpp)
#set_target_properties(segment_service PROPERTIES COMPILE_FLAGS "-O3")

#add_executable(build_segment_info src/build_segment_info.cpp ${HEADERS} ${SOURCES})
#target_link_libraries(build_segment_info ${catkin_LIBRARIES})
#add_dependencies(build_segment_info ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#set_target_properties(build_segment_info PROPERTIES COMPILE_FLAGS "-O3")

#add_executable(possible_map_visualizer src/possible_map_visualizer.cpp ${HEADERS} ${SOURCES})
#target_link_libraries(possible_map_visualizer ${catkin_LIBRARIES})
#add_dependencies(possible_map_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
