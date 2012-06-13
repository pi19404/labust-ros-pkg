cmake_minimum_required (VERSION 2.6)
project(labust_bv)

#Change major version after tagging
set(MAJOR_VERSION 0)
#Change minor version to last revision were it was edited.
set(MINOR_VERSION 59)

#Define output directories.
set(LIBRARY_OUTPUT_PATH $ENV{LIB_ABS_DIR})
set(EXECUTABLE_OUTPUT_PATH $ENV{BIN_ABS_DIR})

#Set compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")

#Depends on Boost >= 1.46 headers
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.46.0 COMPONENTS thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)

#Inseparable from ROS

#set(PR_NAME1 bv_node)
#add_executable(${PR_NAME1} src/bv_node.cpp)
#target_link_libraries(${PR_NAME1} ${Boost_THREAD_LIBRARY})

#set(PR_NAME2 bv_monitor)
#add_executable(${PR_NAME2} src/bv_monitor.cpp)

#install(DIRECTORY include/labust DESTINATION include)
#install(TARGETS ${PR_NAME1} ${PR_NAME2} DESTINATION bin)

#Add library target
#set(PACKAGE_NAME labust-bv-dev)
#set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The BlueView acquistion and monitoring tools.")
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "libbvt-sdk (>=0.59), libboost-dev (>=1.46.0) libboost-thread-dev (>=1.46.0)")
#include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
#include(CPack)
