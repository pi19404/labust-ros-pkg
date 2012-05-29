cmake_minimum_required (VERSION 2.6)
project(labust_moos)

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
find_package(Boost 1.46.0 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#External includes and link
include_directories($ENV{EXTERNAL_INCLUDE_DIRS})
link_directories($ENV{EXTERNAL_LIB_DIRS})
#Internal includes
include_directories(include/)
#Add the external moos location
if ("$ENV{MOOS_HOME}" STREQUAL "")
	message(FATAL_ERROR "MOOS_HOME enviromental variable missing.")
endif ()
include_directories($ENV{MOOS_HOME}/moos/Core)
link_directories($ENV{MOOS_HOME}/build/MOOS/MOOSBin)
set(MOOS_LIBRARIES MOOS MOOSUtility MOOSGen)


set(PR_NAME labust_moos)
file(GLOB SRC src/*.cpp src/*.c)
file(GLOB HPP include/labust/moos/*.hpp include/labust/moos/*.h)
add_library(${PR_NAME} ${SRC} ${HPP})
target_link_libraries(${PR_NAME} ${MOOS_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

install(DIRECTORY include/labust DESTINATION include)
install(TARGETS ${PR_NAME} DESTINATION lib)

#Add library target
set(PACKAGE_NAME liblabust-moos-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The MOOS utilities and libraries for LABUST applications.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-comms-dev (>=0.59)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
