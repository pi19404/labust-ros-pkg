cmake_minimum_required(VERSION 2.8.3)
project(labust_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  roscpp
  snippets
  sensor_msgs
  geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   auv_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES labust_navigation
  CATKIN_DEPENDS auv_msgs roscpp snippets
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(labust_navigation
   src/labust_navigation/KinematicModel.cpp
   src/labust_navigation/XYModel.cpp
   src/labust_navigation/SensorHandlers.cpp
)

## Declare a cpp executable
# add_executable(labust_navigation_node src/labust_navigation_node.cpp)
add_executable(eigen_test src/test/eigen_test.cpp)
add_executable(euler_quat src/test/euler_quat.cpp)
add_executable(quat_to_euler src/test/quat_to_euler.cpp)
add_executable(nav_node2 src/nav_node/nav_node2.cpp)
add_executable(llnode src/latlon_node/LLNode.cpp)
add_executable(static_transform_publisher src/static_transform_publisher/static_transform_publisher.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(labust_navigation_node labust_navigation_generate_messages_cpp)
if (DEFINED catkin_EXPORTED_TARGETS)
 add_dependencies(eigen_test ${catkin_EXPORTED_TARGETS})
 add_dependencies(euler_quat ${catkin_EXPORTED_TARGETS})
 add_dependencies(quat_to_euler ${catkin_EXPORTED_TARGETS})
 add_dependencies(nav_node2 ${catkin_EXPORTED_TARGETS})
 add_dependencies(llnode ${catkin_EXPORTED_TARGETS})
 add_dependencies(static_transform_publisher ${catkin_EXPORTED_TARGETS})
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(labust_navigation
   ${catkin_LIBRARIES}
)

target_link_libraries(eigen_test ${catkin_LIBRARIES} labust_navigation)
target_link_libraries(euler_quat ${catkin_LIBRARIES} labust_navigation)
target_link_libraries(quat_to_euler ${catkin_LIBRARIES})
target_link_libraries(nav_node2 ${catkin_LIBRARIES} labust_navigation)
target_link_libraries(llnode ${catkin_LIBRARIES})
target_link_libraries(static_transform_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS labust_navigation labust_navigation_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
