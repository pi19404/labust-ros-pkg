#!/bin/bash
DATAFILE=caddy_data.tar.bz2
URL=https://www.dropbox.com/s/0i2olo9rm5tz1y8/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
