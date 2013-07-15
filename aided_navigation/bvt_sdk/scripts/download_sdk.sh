#!/bin/bash
DATAFILE=bvt_sdk_linux_x86_5861.tar.gz
URL=http://labust-ros-pkg.googlecode.com/files/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
