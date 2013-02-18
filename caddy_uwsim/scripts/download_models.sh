#!/bin/bash
DATAFILE=caddy_data_v0.1.tar.bz2
URL=http://labust-ros-pkg.googlecode.com/files/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
