#!/bin/bash
#
# Does an out-of-source build of projects.
#
#The trunk directory is one above the script
TRUNK_ABS_DIR="$(cd "$( dirname ${0} )" ; cd .. ; pwd )"
#Remember the desired build, binary, library and package directory.
BUILD_ABS_DIR=${TRUNK_ABS_DIR}/build
BIN_ABS_DIR=${BUILD_ABS_DIR}/bin
LIB_ABS_DIR=${BUILD_ABS_DIR}/lib
PACKAGE_ABS_DIR=${BUILD_ABS_DIR}/packages
#Create the directories.
mkdir -p ${BIN_ABS_DIR} ${LIB_ABS_DIR} ${PACKAGE_ABS_DIR}

#Get together all the include directories
export EXTERNAL_INCLUDE_DIRS=`find ${TRUNK_ABS_DIR} -type d -name "include"`
export EXTERNAL_LIB_DIRS=${BUILD_ABS_DIR}/lib
export CMAKE_CONFIG_DIR=${TRUNK_ABS_DIR}/scripts/config
export BIN_ABS_DIR=${BIN_ABS_DIR}
export LIB_ABS_DIR=${LIB_ABS_DIR}

#List here the project in build order. We can automatically enumerate 
#all the projects but dependecies are not handled. Listing them in order
#will reduce how many time we need to run this script.
UTILS="labust_tools labust_xml labust_drivers labust_comms labust_navcon"
SIM="labust_uvsim"
HARDWARE="videoray seamor bvt_sdk labust_bv"
MOOS="moos_libs"
PROJECT_LIST="${UTILS} ${SIM} ${HARDWARE} ${MOOS}"

for project in ${PROJECT_LIST}
do
	#Find the directory, should contain a CMakeLists.txt file.
	DIRS=`find ${TRUNK_ABS_DIR} -type d -name ${project}`
	for dir in ${DIRS}
	do
		if [ -e ${dir}/CMakeLists.txt ]; then
			#Remember the source and binary directory.
			SRC_ABS_DIR=${dir}
			TARGET_ABS_DIR=${BUILD_ABS_DIR}/${project}
	
			#Create the binary directory and position inside it.
			mkdir -p ${TARGET_ABS_DIR}
			cd ${TARGET_ABS_DIR}

			#Call CMake and compile with make.
			cmake -DCLEAN_BUILD=ON ${SRC_ABS_DIR}
			make ${*}

			#Try to collect the *.deb archives.
			if [ -e ${TARGET_ABS_DIR}/*.deb ] 
			then
				cp ${TARGET_ABS_DIR}/*.deb ${PACKAGE_ABS_DIR}
			fi
		fi
	done
done
