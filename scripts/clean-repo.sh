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
	#Delete the bin directories
	if [ -d bin ] 
	then
		rm -r bin
	fi
	#Delete the lib directories
	if [ -d lib ] 
	then
		rm -r lib
	fi
	#Delete the build directories
	if [ -d build ] 
	then
		rm -r build
	fi
done
exit 0
