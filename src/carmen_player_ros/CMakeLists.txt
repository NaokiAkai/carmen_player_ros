cmake_minimum_required(VERSION 3.0.2)
project(carmen_player_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carmen_player_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(carmen_player_ros src/carmen_player_ros.cpp)
target_link_libraries(carmen_player_ros ${catkin_LIBRARIES})

