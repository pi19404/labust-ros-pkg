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

set(QtComponentsUsed QtCore QtGui QtXml)
find_package(Qt4 COMPONENTS ${QtComponentsUsed})
include_directories(${QT_INCLUDES})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/config)
#find_package(Qwt6 REQUIRED)
#include_directories(${Qwt_INCLUDES})

rosbuild_add_boost_directories()
include_directories(include/)
include_directories(build/)

#common commands for building c++ executables and libraries
set(PR_NAME dynrec_gui)
set(SRC src/DynRecMainWindow.cpp)
set(HPP include/labust/gui/DynRecMainWindow.hpp)
qt4_wrap_ui(UI_HPP ui/DynRecMainWindow.ui)
qt4_wrap_cpp(MOC_HPP include/labust/gui/DynRecMainWindow.hpp)
rosbuild_add_library(${PR_NAME} ${SRC} ${HPP} ${UI_HPP} ${MOC_HPP})
target_link_libraries(${PR_NAME} ${QT_LIBRARIES})

set(PR_NAME2 dynrec_app)
set(SRC src/dynRecMain.cpp src/DynRecMoosApp.cpp)
set(HPP include/labust/moos/DynRecMoosApp.hpp)
rosbuild_add_executable(${PR_NAME2} ${SRC} ${HPP})
target_link_libraries(${PR_NAME2} ${PR_NAME})
rosbuild_link_boost(${PR_NAME2} thread)


