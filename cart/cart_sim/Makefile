download_and_build: downloaded all

DATAFILE=cart_data_v0.1.tar.bz2
URL = http://labust-ros-pkg.googlecode.com/files/${DATAFILE}

downloaded:
	wget -nc --no-check-certificate $(URL) && tar xvf $(DATAFILE) && rm $(DATAFILE)
	touch downloaded

include $(shell rospack find mk)/cmake.mk
