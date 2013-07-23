#!/bin/bash
DATAFILE=PCM-3362_SUSI_release__2010_12_06.tar.gz
URL=https://www.dropbox.com/s/bcqb9wgx1y3wzfc/${DATAFILE}

#if [ ! -f downloaded ];
#then
	mkdir -p lib
	cd lib
	wget -nc --no-check-certificate ${URL} && tar xvf ${DATAFILE} && rm ${DATAFILE}
	echo "Copying REL_Linux_SUSI.h into /usr/local/include..."
	sudo mv REL_Linux_SUSI.H /usr/local/include/REL_Linux_SUSI.h
	echo "Copying libSUSI into /usr/local/lib..."
	sudo rm -f /usr/local/lib/libSUSI*
	sudo mv libSUSI* /usr/local/lib
	sudo ln -s /usr/local/lib/libSUSI-3.02.so /usr/local/lib/libSUSI.so 
	cd ..
	rm -rf lib
#fi
