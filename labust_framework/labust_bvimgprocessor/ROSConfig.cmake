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

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDES})
include_directories(include)


set(PR_NAME1 bvimgprocessor)
set(SRC src/TrackerROI.cpp
	src/ImageProcessing.cpp)
set(HPP include/labust/blueview/TrackerROI.hpp
	include/labust/blueview/ImageProcessing.hpp)
rosbuild_add_library(${PR_NAME1} ${SRC} ${HPP})
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
rosbuild_add_library(${PR_NAME2} ${SRC} ${HPP})
target_link_libraries(${PR_NAME2} ${OpenCV_LIBS} modpbase64)

rosbuild_add_executable(processor_test src/processor_test.cpp)
target_link_libraries(processor_test ${PR_NAME2})

rosbuild_add_executable(bv_processor src/bv_processor.cpp)
target_link_libraries(bv_processor ${PR_NAME1} ${PR_NAME2})
