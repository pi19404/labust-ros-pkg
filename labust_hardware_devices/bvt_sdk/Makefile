all: blueview_sdk

TARBALL = bvt_sdk_4966_i686.tar.gz
TARBALL_URL = https://internal.blueview.com/drop/software/5861/bvt_sdk_linux_x86_5861.tar.gz
UNPACK_CMD = tar xzf
SOURCE_DIR = build/

blueview_sdk: 
	mkdir -p ${SOURCE_DIR}
	cd ${SOURCE_DIR} && wget -nc --no-check-certificate ${TARBALL_URL}
	cd ${SOURCE_DIR} && ${UNPACK_CMD} ${TARBALL}
	cp -r ${SOURCE_DIR}/bvtsdk/include .
	cp -r ${SOURCE_DIR}/bvtsdk/lib .
	cp -r ${SOURCE_DIR}/bvtsdk/colormaps .
	rm -rf ${SOURCE_DIR}/bvtsdk/include
	rm -rf ${SOURCE_DIR}/bvtsdk/lib
	rm -rf ${SOURCE_DIR}/bvtsdk/colormaps
	rm -rf ${SOURCE_DIR}/bvtsdk
clean:
	rm -rf ${SOURCE_DIR}
	rm -rf lib
	rm -rf include
	rm -rf colormaps
