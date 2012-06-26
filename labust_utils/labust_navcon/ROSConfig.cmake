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

set(PR_NAME labust_navcon)
file(GLOB SRC src/*.cpp src/*.c)
file(GLOB HPP include/labust/control/*.hpp 
	include/labust/control/*.h
	include/labust/navigation/*.hpp
	include/Labust/navigation/*.h)
rosbuild_add_library(${PR_NAME} ${SRC} ${HPP})
#Build without plugin hooks.
set_target_properties(${PR_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_WITHOUT_PLUGIN_HOOK")


set(LFC_NAME lfcontrol-plug)
set(SRC src/LFController.cpp)
set(HPP include/labust/control/LFController.hpp)
rosbuild_add_library(${LFC_NAME} ${SRC} ${HPP})

set(HDC_NAME hdcontrol-plug)
set(SRC src/HDController.cpp)
set(HPP include/labust/control/HDController.hpp)
rosbuild_add_library(${HDC_NAME} ${SRC} ${HPP})

set(LFNAV_NAME lfnavigation-plug)
set(SRC src/LFNav.cpp src/LFModel.cpp)
set(HPP include/labust/navigation/LFNav.hpp include/labust/navigation/LFModel.hpp)
rosbuild_add_library(${LFNAV_NAME} ${SRC} ${HPP})

set(KINNAV_NAME kinematicnavigation-plug)
set(SRC src/KinematicModel.cpp)
set(HPP include/labust/navigation/KinematicModel.hpp)
rosbuild_add_library(${KINNAV_NAME} ${SRC} ${HPP})

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#Test code
rosbuild_add_executable(control_test src/test/control_test.cpp)
target_link_libraries(control_test ${PR_NAME})
