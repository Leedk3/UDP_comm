cmake_minimum_required(VERSION 2.8.3)
project(unmanned_sol_udp_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utm_comm_utm_lib
  CATKIN_DEPENDS roscpp std_msgs
)


find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
add_definitions(${GeographicLib_DEFINITIONS})

add_library(utm_comm_utm_lib src/UTM.cpp)
add_dependencies(utm_comm_utm_lib   ${catkin_EXPORTED_TARGETS})
target_link_libraries(utm_comm_utm_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(unmanned_sol_udp_server src/unmanned_ugv_udp_server.cpp)
add_dependencies(unmanned_sol_udp_server   ${catkin_EXPORTED_TARGETS})
target_link_libraries(unmanned_sol_udp_server
  ${catkin_LIBRARIES}
  utm_comm_utm_lib
)

add_executable(etri_udp_server src/etri_udp_server.cpp)
add_dependencies(etri_udp_server   ${catkin_EXPORTED_TARGETS})
target_link_libraries(etri_udp_server
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  utm_comm_utm_lib
)

link_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
