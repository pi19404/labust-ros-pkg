#!/bin/bash
DATAFILE=ldtravocean_data.tar.bz2
URL=https://www.dropbox.com/s/znjh8rittxow80b/${DATAFILE}

if [ ! -f downloaded ];
then
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	touch downloaded
fi
