download_and_build: downloaded all

DATAFILE=cart_data.tar.bz2
URL = https://www.dropbox.com/s/yr44t0h6d1h8i1w/${DATAFILE}

downloaded:
	wget -nc --no-check-certificate $(URL) && tar xvf $(DATAFILE) && rm $(DATAFILE)
	touch downloaded

include $(shell rospack find mk)/cmake.mk
