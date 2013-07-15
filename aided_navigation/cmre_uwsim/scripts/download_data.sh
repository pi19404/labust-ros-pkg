#!/bin/bash
DATAFILE=gemellina.tar.bz2
URL=https://www.dropbox.com/s/z43rxemg4ubumzj/${DATAFILE}
ROS_PACKAGE_NAME=cmre_uwsim

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
