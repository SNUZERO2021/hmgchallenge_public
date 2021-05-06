cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

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

#add_library(quartic_polynomial src/quartic_polynomial.cpp)
#add_library(quintic_polynomial src/quintic_polynomial.cpp)
#add_library(car src/car.cpp)
#add_library(cubic_spline_1d_p src/cubic_spline_1d.cpp)
#add_library(cubic_spline_2d_p src/cubic_spline_2d.cpp)
#add_library(obstacle src/obstacle.cpp)
#add_library(frenet_path src/frenet_path.cpp)
#add_library(frenet_optimal_trajectory src/frenet_optimal_trajectory.cpp)

set (SOURCES 
 ${CMAKE_CURRENT_SOURCE_DIR}/src/quartic_polynomial.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/quintic_polynomial.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/car.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/src/cubic_spline_1d.cpp
 #${CMAKE_CURRENT_SOURCE_DIR}/src/cubic_spline_2d.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/obstacle.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/frenet_path.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/frenet_optimal_trajectory.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit.cpp
)

set (HEADERS
 ${CMAKE_CURRENT_SOURCE_DIR}/include/quartic_polynomial.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/quintic_polynomial.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/car.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_spline_1d.h
 #${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_spline_2d.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/obstacle.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/cpp_struct.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/frenet_path.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/frenet_optimal_trajectory.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/pure_pursuit.h
)

#add_executable(frenet_optimal_trajectory_main src/main.cpp ${HEADERS} ${SOURCES})
#add_dependencies(frenet_optimal_trajectory_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(frenet_optimal_trajectory_main ${catkin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
#set_target_properties(frenet_optimal_trajectory_main PROPERTIES COMPILE_FLAGS "-O3")

add_executable(path_planning_node src/path_planning_node.cpp ${HEADERS} ${SOURCES})
add_dependencies(path_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_node ${catkin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
