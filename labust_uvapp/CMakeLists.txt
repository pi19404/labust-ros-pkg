cmake_minimum_required(VERSION 2.8.3)
project(labust_uvapp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS labust_base std_msgs auv_msgs nav_msgs roscpp dynamic_reconfigure sensor_msgs
	message_generation tf orocos_kdl)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
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
add_service_files(
   FILES
   ConfigureVelocityController.srv
   EnableControl.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################################
## Dynamic reconfigure
###################################################
generate_dynamic_reconfigure_options(cfg/VelCon.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES labust_uvapp
   CATKIN_DEPENDS labust_base auv_msgs nav_msgs std_msgs roscpp dynamic_reconfigure sensor_msgs tf orocos_kdl
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(labust_uvapp
#   src/${PROJECT_NAME}/labust_uvapp.cpp
# )

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## Declare a cpp executable
# add_executable(labust_uvapp_node src/labust_uvapp_node.cpp)
set(PR_NAME velocity_control)
add_executable(${PR_NAME} 
	src/velocity_control.cpp 
	src/VelocityControl.cpp
	include/labust/control/VelocityControl.hpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME lf_control)
add_executable(${PR_NAME} 
	src/lf_control.cpp 
	src/LFControl.cpp
	include/labust/control/LFControl.hpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME dp_control)
add_executable(${PR_NAME} 
	src/dp_control.cpp 
	src/DPControl.cpp
	include/labust/control/DPControl.hpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME vt_control)
add_executable(${PR_NAME} 
	src/vt_control.cpp 
	src/VirtualTarget.cpp
	include/labust/control/VirtualTarget.hpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME hdg_control)
add_executable(${PR_NAME} 
	src/hdg_control.cpp 
	src/HeadingControl.cpp
	include/labust/control/HeadingControl.hpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME hdg_test)
add_executable(${PR_NAME} 
	src/test/hdg_test.cpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

set(PR_NAME lf_test)
add_executable(${PR_NAME} 
	src/test/lf_test.cpp)
target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

#set(PR_NAME labust-uvapp)
#add_library(${PR_NAME} 
#	src/VelocityControl.cpp
#	include/labust/control/VelocityControl.hpp)
#target_link_libraries(${PR_NAME} ${catkin_LIBRARIES} labust_navcon)

## Add dependencies to the executable
#add_dependencies(velocity_control ${PROJECT_NAME}_gencfg)
add_dependencies(dp_control ${PROJECT_NAME}_gencpp)
add_dependencies(vt_control ${PROJECT_NAME}_gencpp)
add_dependencies(lf_control ${PROJECT_NAME}_gencpp)
#add_dependencies(velocity_control ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(labust_uvapp_node
#   ${catkin_LIBRARIES}
# )

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
#install(TARGETS velocity_control
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_uvapp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
