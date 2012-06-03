cmake_minimum_required (VERSION 2.6)
project(labust_comms)

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
find_package(Boost 1.46.0 REQUIRED COMPONENTS system thread regex)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)

set(PROJECT_NAME labust_comms)
file(GLOB SRC src/UUVModem.cpp src/*.c)
file(GLOB HPP include/labust/comms/*.hpp
	include/labust/comms/*.h)
add_library(${PROJECT_NAME} SHARED ${SRC} ${HPP})
target_link_libraries(${PROJECT_NAME} labust_xml ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

#Configure installation
install(DIRECTORY include/labust DESTINATION include/)

#Configure package
set(PACKAGE_NAME liblabust-comms-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Helper library for communication protocols and devices.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev (>= 1.46), libboost-system-dev (>= 1.46), liblabust-xml-dev (>= 0.59)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
