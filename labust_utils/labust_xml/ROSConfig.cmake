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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#Find boost and xml2
rosbuild_add_boost_directories()
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})

#Add library target
set(PROJECT_NAME labust_xml)
file(GLOB SRC src/*.cpp src/*.c)
file(GLOB HPP include/labust/xml/*.hpp include/labust/xml/*.h)

rosbuild_add_library(${PROJECT_NAME} ${SRC} ${HPP})
target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES})

#Library test code
#Gyros
rosbuild_add_executable(gyros_test src/test/gyros_test.cpp)
target_link_libraries(gyros_test ${PROJECT_NAME})
#XML
rosbuild_add_executable(xml_test src/test/xml_test.cpp)
target_link_libraries(xml_test ${PROJECT_NAME})
