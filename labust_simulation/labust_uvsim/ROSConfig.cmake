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

rosbuild_add_boost_directories()
#The labust_model target definition
set(HPP include/labust/simulation/VehicleModel6DOF.hpp
	include/labust/simulation/NoiseModel.hpp)
set(SRC src/VehicleModel6DOF.cpp src/NoiseModel.cpp)

SET(PR_NAME labust_model)
rosbuild_add_library(${PR_NAME} ${SRC} ${HPP})
rosbuild_link_boost(${PR_NAME} random)

#The uvsim target definition
set(PR_NAME2 uvsim-plug)
rosbuild_add_library(${PR_NAME2} src/UVSim.cpp include/labust/simulation/UVSim.hpp)
target_link_libraries(${PR_NAME2} ${PR_NAME})

#The general UDP wrapped simulator target definition
set(PR_NAME3 udpsim)
rosbuild_add_executable(${PR_NAME3} src/UDPSimMain.cpp src/UDPSim.cpp include/labust/simulation/UDPSim.hpp) 
target_link_libraries(${PR_NAME3} ${PR_NAME2})
rosbuild_link_boost(${PR_NAME3} system thread)

