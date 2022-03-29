cmake_minimum_required(VERSION 3.1)

project(safety_node1)


find_package(catkin REQUIRED COMPONENTS roslib
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    ackermann_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs)
######################################
# Compile the library >
######################################

# Include catkin_libraries
catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
  )
  set(LIBS ${LIBS} ${catkin_LIBRARIES})
  	include_directories(include ${catkin_INCLUDE_DIRS})

# Add the nodes
file(GLOB NODE_SRC_FILES src/*.cpp)
foreach(_node_file ${NODE_SRC_FILES})
  get_filename_component(_node_name ${_node_file} NAME_WE)
  add_executable(${_node_name} ${_node_file})
  target_link_libraries(${_node_name} ${LIBS})

endforeach()


######################################
# < End ROS
######################################


