cmake_minimum_required(VERSION 2.8.3)
project(m7_tracking)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
  image_transport
  cv_bridge
  message_generation
  cmake_modules
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)


generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS eigen system_lib opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(m7_tracker_params include/m7_tracking/Params.h include/m7_tracking/Params.cpp)
set_target_properties(m7_tracker_params PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(m7_tracker_params ${catkin_LIBRARIES})

add_library(m7_tracker include/m7_tracking/Tracker.cpp)
target_link_libraries(m7_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} m7_tracker_params)

add_executable(roomba_tracker src/roomba_tracking.cpp)
target_link_libraries(roomba_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} m7_tracker_params m7_tracker)

