#!/bin/bash
DATAFILE=cart_data.tar.bz2
URL=https://www.dropbox.com/s/yr44t0h6d1h8i1w/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
