cmake_minimum_required(VERSION 2.8.3)
project(bvt_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	nodelet
	dynamic_reconfigure
	aidnav_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED serialization thread system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  MBSonar.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   SetRange.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs 
#   sensor_msgs
#   geometry_msgs
#)

###################################################
## Dynamic reconfigure
###################################################
generate_dynamic_reconfigure_options(cfg/BVSonar.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/
  #LIBRARIES bvt_sdk
  CATKIN_DEPENDS roscpp rospy aidnav_msgs nodelet
  #DEPENDS message_runtime rospy roscpp
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
## Specify additional locations of header files
include_directories(include include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(bvt_sdk
#   src/${PROJECT_NAME}/bvt_sdk.cpp
# )

## Declare a cpp executable
#add_executable(blueview_node src/BVSonarRos.cpp src/blueview_node.cpp)
add_library(blueview-nodelet src/BVSonarRos.cpp)

## Add dependencies to the executable
add_dependencies(blueview-nodelet ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(blueview-nodelet bvtsdk ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bvt_sdk bvt_sdk_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(DIRECTORY 
#	bvtsdk/include/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/bvtsdk
#)

#install(DIRECTORY
#   bvtsdk/colormaps
#   bvtsdk/data
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(FILES
#   bvtsdk/lib/*
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bvt_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
