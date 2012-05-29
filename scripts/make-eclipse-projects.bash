#!/bin/bash                                                            
if [ $# -ne 0 ]; then
	for project in $@ 
	do
		DIR=`find .. -type d -name ${project}`
		for dir in ${DIR}
		do
			if [ -e "${dir}/Makefile" ]; then
				(cd ${dir}; make eclipse-project)
			fi
		done
	done
else
	echo "Generating eclipse projects for all ROS projects in this directory"
	for MKFILE in `find .. -name Makefile`; do
  	  DIR=`dirname $MKFILE`
	    echo $DIR
	    (cd $DIR; make eclipse-project)
	done
fi
