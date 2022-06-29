cmake_minimum_required(VERSION 3.0.2)
project(follow_the_gap)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


