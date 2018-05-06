cmake_minimum_required(VERSION 2.8.3)
project(ariac_examples)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_visualization
  trajectory_msgs
  std_msgs
  message_generation
)
add_service_files(
  FILES
  coor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)
## Declare a C++ executable

add_executable(js src/final.cpp)
add_dependencies(js ${catkin_EXPORTED_TARGETS})
target_link_libraries(js ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
##install(PROGRAMS script/ariac_example_node.py
##  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

## Mark executables and/or libraries for installation
install(TARGETS js
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
# config/sample_gear_conf.yaml
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

