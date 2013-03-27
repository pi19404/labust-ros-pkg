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

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

SET(PR_NAME cart2_node)
rosbuild_add_executable(${PR_NAME} src/cart2_node.cpp)
target_link_libraries(${PR_NAME} labust_uvapp labust_xml labust_navcon)

SET(PR_NAME vr_node)
rosbuild_add_executable(${PR_NAME} src/vr_node.cpp)
target_link_libraries(${PR_NAME} labust_uvapp labust_xml labust_navcon)

SET(PR_NAME velocity_demo)
rosbuild_add_executable(${PR_NAME} src/velocity_demo.cpp)
#target_link_libraries(${PR_NAME} labust_uvapp labust_xml labust_navcon)
