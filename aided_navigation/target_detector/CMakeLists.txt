cmake_minimum_required(VERSION 2.8.3)
project(target_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS aidnav_msgs labust_base roscpp rospy cv_bridge)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

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
#  LIBRARIES target_detector
  CATKIN_DEPENDS aidnav_msgs cv_bridge roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDES})

set(PR_NAME1 bvimgprocessor)
set(SRC src/TrackerROI.cpp
	src/ImageProcessing.cpp)
set(HPP include/labust/blueview/TrackerROI.hpp
	include/labust/blueview/ImageProcessing.hpp)
add_library(${PR_NAME1} ${SRC} ${HPP})
target_link_libraries(${PR_NAME1} ${OpenCV_LIBS} modpbase64)

set(PR_NAME2 bvprocchain)
set(SRC src/TrackerROI.cpp
	src/PrefilterPolicy.cpp
	src/ThresholdPolicy.cpp
	src/LabelPolicy.cpp
	src/AssociatePolicy.cpp
	src/CConverter.cpp)
SET(HPP include/labust/blueview/trackerfwd.hpp
		include/labust/blueview/TrackerROI.hpp
		include/labust/blueview/PrefilterPolicy.hpp
		include/labust/blueview/ThresholdPolicy.hpp
		include/labust/blueview/LabelPolicy.hpp
		include/labust/blueview/AssociatePolicy.hpp
		include/labust/blueview/ProcessingChain.hpp)
add_library(${PR_NAME2} ${SRC} ${HPP})
add_dependencies(${PR_NAME2} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PR_NAME2} ${OpenCV_LIBS} modpbase64 labust_xml labust_navcon)

add_executable(processor_test src/processor_test.cpp)
add_dependencies(processor_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(processor_test ${PR_NAME2})

add_executable(bv_processor src/bv_processor.cpp)
add_dependencies(bv_processor ${catkin_EXPORTED_TARGETS})
target_link_libraries(bv_processor ${PR_NAME1} ${PR_NAME2} ${catkin_LIBRARIES})

add_executable(bv_monitor src/bv_monitor.cpp)
add_dependencies(bv_monitor ${catkin_EXPORTED_TARGETS})
target_link_libraries(bv_monitor ${catkin_LIBRARIES})


## Declare a cpp library
# add_library(target_detector
#   src/${PROJECT_NAME}/target_detector.cpp
# )

## Declare a cpp executable
# add_executable(target_detector_node src/target_detector_node.cpp)

## Add dependencies to the executable
# add_dependencies(target_detector_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(target_detector_node
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
# install(TARGETS target_detector target_detector_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_target_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
