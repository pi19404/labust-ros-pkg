cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#set(PR_NAME vehicleNode)
#set(SRC src/vehicleNode.cpp src/VehicleApp.cpp)
#set(HPP include/labust/vehicles/VehicleApp.hpp)
#rosbuild_add_executable(${PR_NAME} ${SRC} ${HPP})
#target_link_libraries(${PR_NAME} labust_xml)

set(PR_NAME velocity_control)
rosbuild_add_executable(${PR_NAME} 
	src/velocity_control.cpp 
	src/VelocityControl.cpp
	include/labust/control/VelocityControl.hpp)
target_link_libraries(${PR_NAME} labust_navcon)

rosbuild_add_executable(identificationNode src/identificationNode.cpp)
target_link_libraries(identificationNode labust_xml labust_navcon)
