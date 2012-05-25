cmake_minimum_required (VERSION 2.6)
project(labust_navcon)

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
find_package(Boost 1.46.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)

set(PROJECT_NAME labust_navcon)
file(GLOB SRC src/*.cpp src/*.c)
file(GLOB HPP include/labust/navigation/*.hpp include/labust/navigation/*.h 
	include/labust/control/*.hpp include/labust/control/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC} ${HPP})
target_link_libraries(${PROJECT_NAME} labust_xml)

install(DIRECTORY include/labust DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

#Add library target
set(PACKAGE_NAME liblabust-navcon-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Navigation and control utilities for LABUST framework applications.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-tools-dev (>=0.59), liblabust-xml-dev (>=0.59), liblabust-drivers-xml (>=0.59), libboost-dev (>=1.46.0)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)

#Library test code
#Gyros
add_executable(control_test src/test/control_test.cpp)
target_link_libraries(control_test ${PROJECT_NAME})
