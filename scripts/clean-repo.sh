#!/bin/bash
#
# Does an out-of-source build of projects.
#
#The trunk directory is one above the script
TRUNK_ABS_DIR="$(cd "$( dirname ${0} )" ; cd .. ; pwd )"
#Remember the desired build, binary, library and package directory.

#Remove build dir
rm -rf ${TRUNK_ABS_DIR}/build

DIRS=`find ${TRUNK_ABS_DIR} -type f -name "CMakeLists.txt" -printf '%h\n' | sort -u`

for dir in ${DIRS}
do
	#Go to directory.
	cd ${dir}
	#Clean it.
	make clean
	#Delete the build,bin,lib directories
	rm -rf bin lib build
	#Delete eclipse and cmake files
	rm -rf .cproject .project cmake_install.cmake .pydevproject
done
exit 0
