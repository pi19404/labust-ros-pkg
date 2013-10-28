cmake_minimum_required(VERSION 2.8.3)
project(ldtravocean)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    snippets 
    auv_msgs 
    tf
    kdl_conversions
    labust_navigation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ldtravocean
   CATKIN_DEPENDS roscpp auv_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
execute_process(COMMAND sh scripts/download_models.sh 
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ldtravocean
#   src/${PROJECT_NAME}/ldtravocean.cpp
# )

## Declare a cpp executable
#add_executable(model_node src/model_node.cpp)

## Declare a cpp executable
add_executable(ldnav_node src/nav_node.cpp src/LDTravModel.cpp
	include/labust/navigation/LDTravModel.hpp)

## Add dependencies to the executable
# add_dependencies(ldtravocean_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
#target_link_libraries(model_node ${catkin_LIBRARIES} labust_xml uvsim-plug)

target_link_libraries(ldnav_node ${catkin_LIBRARIES})

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
# install(TARGETS ldtravocean ldtravocean_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ldtravocean.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
