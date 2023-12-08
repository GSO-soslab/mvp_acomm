cmake_minimum_required(VERSION 3.5.1)
project(alpha_acomms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  mvp_msgs
  nav_msgs
  message_filters
  geometry_msgs
  geographic_msgs
  robot_localization
  serial
)

find_package(Protobuf REQUIRED)

find_package(goby 3.1 REQUIRED)

find_package(dccl REQUIRED)


# build the protobuf messages
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/proto/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(proto STATIC ${ProtoSources} ${ProtoHeaders})
target_link_libraries(proto ${PROTOBUF_LIBRARY})

include_directories( ${CMAKE_BINARY_DIR} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS 
  # LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs geographic_msgs mvp_msgs
  # DEPENDS
)

###########
## Build ##
###########

include_directories(
  include/seatrac_driver
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/seatrac_modem.cpp
  include/seatrac_driver/seatrac_driver.cpp
)

add_executable(alpha_topside
  src/seatrac_usbl.cpp
  include/seatrac_driver/seatrac_driver.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(alpha_topside ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  proto
  goby
  dccl
)

target_link_libraries(alpha_topside
  ${catkin_LIBRARIES}
  proto
  goby
  dccl
)

