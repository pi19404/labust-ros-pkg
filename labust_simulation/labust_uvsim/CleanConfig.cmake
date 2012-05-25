cmake_minimum_required (VERSION 2.6)
project(labust_uvsim)

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

set(PR_NAME labust_model)
set(SRC src/VehicleModel6DOF.cpp src/NoiseModel.cpp)
set(HPP include/labust/simulation/NoiseModel.hpp 
	include/labust/simulation/VehicleModel6DOF.hpp)

add_library(${PR_NAME} ${SRC} ${HPP})
target_link_libraries(${PR_NAME} labust_xml)

#The uvsim-plug target
set(PR_NAME2 uvsim-plug)
add_library(${PR_NAME2} SHARED src/UVSim.cpp include/labust/simulation/UVSim.hpp)
target_link_libraries(${PR_NAME2} ${PR_NAME})

#The udpsim target.
set(PR_NAME3 udpsim)
add_executable(${PR_NAME3} src/UDPSimMain.cpp src/UDPSim.cpp include/labust/simulation/UDPSim.hpp)
target_link_libraries(${PR_NAME3} ${PR_NAME2} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

install(DIRECTORY include/labust DESTINATION include)
install(TARGETS ${PR_NAME} ${PR_NAME2} DESTINATION lib)
install(TARGETS ${PR_NAME3} DESTINATION bin)

#Add library target
set(PACKAGE_NAME liblabust-uvsim-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The vehicle simulation models and applicatios.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-tools-dev (>=0.59), liblabust-xml-dev (>=0.59)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
