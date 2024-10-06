cmake_minimum_required(VERSION 3.8)
project(mvp_acomm_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mvp_acomm_interfaces REQUIRED)


find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  geographic_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  mvp_acomm_interfaces
)
include_directories(include)
set(library_name ${PROJECT_NAME})

#Add library
add_library(${library_name} SHARED 
  include/acomm_geopoint.cpp
)

ament_target_dependencies(
  ${library_name} 
  ${dependencies}
  )


#Add exe
add_executable(acomm_geopoint_node src/acomm_geopoint_node.cpp)
target_link_libraries(acomm_geopoint_node ${library_name} ${GeographicLib_LIBRARIES})
ament_target_dependencies(acomm_geopoint_node ${dependencies})


# ##install target
install(TARGETS acomm_geopoint_node ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

##launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()