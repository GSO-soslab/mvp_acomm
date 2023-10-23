cmake_minimum_required(VERSION 3.5.1)
project(seatrac)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mvp_msgs
  seatrac_driver
)

find_package(dccl REQUIRED)

find_package(Protobuf REQUIRED)

find_package(goby 3.0 REQUIRED)

# include_directories("${GOBY_INCLUDE_DIR}")
# protobuf_include_dirs("${GOBY_INCLUDE_DIR}")

if(NOT DEFINED DCCL_INCLUDE_DIR)
  #for DCCL 3.0.3 and newer
  get_target_property(DCCL_INCLUDE_DIR dccl INTERFACE_INCLUDE_DIRECTORIES)
endif()

if(DCCL_INCLUDE_DIR)
  message(STATUS "\tIncluding DCCL protobuf dir: ${DCCL_INCLUDE_DIR}")
  include_directories("${DCCL_INCLUDE_DIR}")
  #protobuf_include_dirs("${DCCL_INCLUDE_DIR}")
endif()

# build the protobuf messages
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/proto/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(proto STATIC ${ProtoSources} ${ProtoHeaders})
target_link_libraries(proto ${PROTOBUF_LIBRARY})

message( STATUS ${CMAKE_BINARY_DIR} )
include_directories( ${CMAKE_BINARY_DIR} )

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files 
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR} goby_msgs.proto)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/alpha_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(seatrac_modem src/seatrac_modem.cpp)
add_executable(seatrac_usbl src/seatrac_usbl.cpp)
add_executable(seatrac_test_driver src/test_driver.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(seatrac_modem ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(seatrac_usbl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(seatrac_test_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(seatrac_usbl
  ${catkin_LIBRARIES}
  dccl
  proto
  goby
)
target_link_libraries(seatrac_test_driver
  ${catkin_LIBRARIES}
  dccl
  proto
  goby
)
