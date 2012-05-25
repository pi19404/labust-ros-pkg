cmake_minimum_required (VERSION 2.6)
project(labust_comms)

#Change major version after tagging
set(MAJOR_VERSION 0)
#Change minor version to last revision were it was edited.
set(MINOR_VERSION 59)

#Check if Boost is installed
find_package(Boost COMPONENTS system REQUIRED)

#Configure installation
install(DIRECTORY include/labust DESTINATION include/)

#Configure package
set(PACKAGE_NAME liblabust-comms-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Helper library for communication protocols and devices.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev (>= 1.46), libboost-system-dev (>= 1.46), liblabust-xml-dev (>= 0.59)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
