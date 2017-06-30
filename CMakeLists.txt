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

add_library(tracker include/m7_tracking/tracker.cpp)
add_library(lidar_tracking include/m7_tracking/rplidar.cpp)
add_library(roomba include/m7_tracking/ground_vehicle.cpp)
add_library(controller include/m7_tracking/controller.cpp)
add_executable(roomba_tracker src/roomba_tracking.cpp)

target_link_libraries(tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    ${Eigen_LIBRARIES})
target_link_libraries(lidar_tracking ${catkin_LIBRARIES})
target_link_libraries(roomba_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    ${Eigen_LIBRARIES} tracker)
target_link_libraries(roomba ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
