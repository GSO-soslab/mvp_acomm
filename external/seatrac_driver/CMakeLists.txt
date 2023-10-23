cmake_minimum_required(VERSION 3.0.2)
project(seatrac_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seatrac_driver
#  CATKIN_DEPENDS roscpp mvp_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/seatrac_driver.cpp
)

