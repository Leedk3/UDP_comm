cmake_minimum_required(VERSION 2.8.3)
project(unmanned_sol_udp_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)
add_executable(unmanned_sol_udp_client src/unmanned_ugv_udp_client.cpp)
target_link_libraries(unmanned_sol_udp_client
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(etri_udp_client src/etri_udp_client.cpp)
target_link_libraries(etri_udp_client
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

