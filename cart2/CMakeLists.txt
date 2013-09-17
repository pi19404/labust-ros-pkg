cmake_minimum_required(VERSION 2.8.3)
project(cart2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS auv_msgs labust_base roscpp rospy sensor_msgs std_msgs tf diagnostic_msgs labust_uvapp
	message_generation dynamic_reconfigure snippets)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HLMessage.msg
  ImuInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetHLMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

###################################################
## Dynamic reconfigure
###################################################
generate_dynamic_reconfigure_options(cfg/RemoteCommand.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cart2
   CATKIN_DEPENDS auv_msgs labust_base roscpp rospy sensor_msgs std_msgs tf diagnostic_msgs message_runtime dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########
execute_process(COMMAND sh scripts/download_models.sh 
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(cart2
#   src/${PROJECT_NAME}/cart2.cpp
# )

## Declare a cpp executable
add_executable(radio_node src/radio_node.cpp)
add_executable(cart2_node src/cart2_main.cpp)
add_executable(imu_node src/imu_node.cpp)
add_executable(hlmanager src/hl_node.cpp src/HLManager.cpp
	include/labust/control/HLManager.hpp)
#add_executable(vr_node src/vr_node.cpp)

add_executable(topside_radio src/topside_radio.cpp src/TopsideRadio.cpp src/crc16.cpp
	include/labust/control/TopsideRadio.hpp
	include/labust/control/crc16.h)
	
add_executable(cnr_radio src/cnr_remote.cpp src/CNRRemoteRadio.cpp
	src/crc16.cpp
	include/labust/control/CNRRemoteRadio.hpp
	include/labust/control/crc16.h)
	
add_executable(bench_radio src/bench_radio.cpp src/BenchRadio.cpp
	include/labust/control/BenchRadio.hpp)
	
add_executable(coop_node src/coop_node.cpp)
add_executable(repost_node src/position_repost.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(hlmanager cart2_generate_messages_cpp)
add_dependencies(imu_node cart2_generate_messages_cpp)
#add_dependencies(topside_radio cart2_gencfg)
#add_dependencies(topside_radio cart2_generate_messages_cpp)
#add_dependencies(cnr_radio cart2_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(topside_radio
   ${Boost_SERIALIZATION_LIBRARY}
   ${Boost_REGEX_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(cnr_radio
   ${Boost_SERIALIZATION_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(coop_node
   ${Boost_SYSTEM_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(repost_node
   ${Boost_SYSTEM_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(bench_radio
   ${Boost_SERIALIZATION_LIBRARY}
   ${Boost_REGEX_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(radio_node
   ${Boost_SERIALIZATION_LIBRARY}
   ${Boost_REGEX_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(cart2_node
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_THREAD_LIBRARY}
)

target_link_libraries(imu_node
   ${catkin_LIBRARIES}
)

target_link_libraries(hlmanager
   ${catkin_LIBRARIES}
)

#target_link_libraries(vr_node
#   ${catkin_LIBRARIES}
#   ${Boost_SYSTEM_LIBRARY}
#   videoray-plug
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cart2 cart2_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cart2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
