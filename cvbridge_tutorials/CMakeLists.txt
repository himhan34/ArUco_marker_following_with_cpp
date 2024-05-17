cmake_minimum_required(VERSION 3.0.2)
project(cvbridge_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  geometry_msgs
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cvbridge_tutorials
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs geometry_msgs image_transport
  DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(clahe src/Clahe.cpp)
add_executable(cv_example src/cv_example.cpp)
add_executable(marker_detection src/marker_detection.cpp)
add_executable(teleop src/teleop.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(clahe
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(cv_example
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(marker_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(teleop
  ${catkin_LIBRARIES}
)
