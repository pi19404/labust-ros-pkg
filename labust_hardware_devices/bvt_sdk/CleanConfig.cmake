cmake_minimum_required (VERSION 2.6)
project(bvt_sdk)

#Change major version after tagging
set(MAJOR_VERSION 0)
#Change minor version to last revision were it was edited.
set(MINOR_VERSION 1)

execute_process(COMMAND make WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

#Configure installation
install(DIRECTORY include/bvt_c DESTINATION include/)
install(DIRECTORY include/bvt_cpp DESTINATION include/)
install(FILES include/bvt_sdk.h DESTINATION include/)
file(GLOB BVTSDK_LIB lib/*)
install(FILES ${BVTSDK_LIB} DESTINATION lib)
install(DIRECTORY colormaps DESTINATION share/bvt_sdk)

#Configure package
set(PACKAGE_NAME libbvt-sdk)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The installation of the BlueView third-party library.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
include($ENV{CMAKE_CONFIG_DIR}/CPackConfig.cmake)
include(CPack)
