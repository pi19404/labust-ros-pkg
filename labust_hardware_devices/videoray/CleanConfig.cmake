cmake_minimum_required (VERSION 2.6)
project(videoray)

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
find_package(Boost 1.46.0 COMPONENTS system thread date_time REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)

set(PROJECT_NAME videoray-plug)
file(GLOB SRC src/*.cpp src/*.c)
file(GLOB HPP include/labust/vehicles/*.hpp include/labust/vehicles/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC} ${HPP})
target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} labust_xml)

install(DIRECTORY include/labust DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

#Add library target
set(PACKAGE_NAME libvideoray-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "VideoRay PRO vehicle driver plug-in.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-comms-dev (>=0.59), liblabust-tools-dev (>=0.59), liblabust-xml-dev (>=0.59), libboost-dev (>=1.46), libboost-system-dev (>=1.46)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)

#Library test code
add_executable(videoray_test src/test/vr_test.cpp)
target_link_libraries(videoray_test ${PROJECT_NAME} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
