#!/bin/bash
DATAFILE=acoustic_vr_media.tar.bz2
URL=https://www.dropbox.com/s/kicfwpw1sf3hiqr/${DATAFILE}

if [ ! -f downloaded ];
then
	cd data/
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	cd ..
	touch downloaded
fi
