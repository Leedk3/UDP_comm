cmake_minimum_required(VERSION 2.8.3)
project(unmanned_sol_udp_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(unmanned_sol_udp_server src/etri_ugv_udp_server.cpp)

target_link_libraries(unmanned_sol_udp_server
  ${catkin_LIBRARIES}
)
