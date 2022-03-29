cmake_minimum_required(VERSION 3.1)

project(safety_node2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf2
    tf2_ros
    tf2_geometry_msgs
    ackermann_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs)

##catkin_python_setup()

# Include catkin_libraries
catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
  )
  
set(LIBS ${LIBS} ${catkin_LIBRARIES})
include_directories(include ${catkin_INCLUDE_DIRS})

## add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/safety_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB NODE_SRC_FILES src/*.py)
foreach(_node_file ${NODE_SRC_FILES})
  get_filename_component(_node_name ${_node_file} NAME_WE)
  #add_executable(${_node_name} ${_node_file})
  target_link_libraries(${_node_name} ${LIBS})

endforeach()
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
## install(TARGETS ${PROJECT_NAME}
##   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##   )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_safety_node2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
