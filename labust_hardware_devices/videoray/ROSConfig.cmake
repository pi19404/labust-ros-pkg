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

file(GLOB SRC src/*.c src/*.cpp)
file(GLOB HPP include/labust/vehicles/*.h include/labust/vehicles/*.hpp)

set(PR_NAME videoray-plug)
rosbuild_add_library(${PR_NAME} ${SRC} ${HPP})
rosbuild_link_boost(${PR_NAME} system)

#Test program
rosbuild_add_executable(videoray_test src/test/vr_test.cpp)
target_link_libraries(videoray_test ${PR_NAME})
rosbuild_link_boost(videoray_test date_time thread)
