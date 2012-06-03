cmake_minimum_required (VERSION 2.6)
project(seamor)

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
find_package(Boost 1.46.0 COMPONENTS system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)

set(PR_NAME1 seamor-plug)
set(SRC src/SeamorDriver.cpp
	src/SeamorCommands.cpp)
set(HPP include/labust/vehicles/SeamorDriver.h
	include/labust/vehicles/SeamorCommands.h)
add_library(${PR_NAME1} SHARED ${SRC} ${HPP})
target_link_libraries(${PR_NAME1} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} labust_xml)

set(PR_NAME2 seamor2-plug)
set(SRC src/Seamor.cpp
	src/SeamorComms.cpp)
set(HPP include/labust/vehicles/Seamor.hpp
	include/labust/vehicles/SeamorComms.hpp)
add_library(${PR_NAME2} SHARED ${SRC} ${HPP})
target_link_libraries(${PR_NAME1} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} labust_xml)

install(DIRECTORY include/labust DESTINATION include)
install(TARGETS ${PR_NAME1} ${PR_NAME2} DESTINATION lib)

#Add library target
set(PACKAGE_NAME libseamor-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Seamor vehicle driver plug-in.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-comms-dev (>= 0.59), liblabust-tools-dev (>=0.59), liblabust-xml-dev (>=0.59), libboost-dev (>=1.46), libboost-system-dev (>=1.46) libboost-thread-dev (>=1.46)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)

#Library test code
add_executable(seamor_test src/seamor_test.cpp)
target_link_libraries(seamor_test ${PR_NAME1} ${PR_NAME2})