#!/bin/bash
DATAFILE=bvt_sdk_6257-ubuntu32bit.tar.gz
URL=https://www.dropbox.com/s/ptj46xbkucq9jou/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
