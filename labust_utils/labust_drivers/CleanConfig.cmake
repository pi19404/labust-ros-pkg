cmake_minimum_required (VERSION 2.6)
project(labust_drivers)

#Change major version after tagging
set(MAJOR_VERSION 0)
#Change minor version to last revision were it was edited.
set(MINOR_VERSION 59)

#Check if Boost is installed
find_package(Boost)

#Configure installation
install(DIRECTORY include/labust DESTINATION include/)

#Configure package
set(PACKAGE_NAME liblabust-drivers-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Abstraction layer drivers for the labust control and navigation frameworks.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "liblabust-tools-dev (>= 0.59), liblabust-xml-dev (>= 0.59), libboost-dev (>= 1.46)")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
