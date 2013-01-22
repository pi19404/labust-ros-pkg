all: blueview_sdk

TARBALL = bvt_sdk_linux_x86_5861.tar.gz
URL = http://labust-ros-pkg.googlecode.com/files/bvt_sdk_linux_x86_5861.tar.gz
UNPACK_CMD = tar xzf
SOURCE_DIR = build/

blueview_sdk:
	wget -nc --no-check-certificate ${URL}
	mkdir -p ${SOURCE_DIR}
	cd ${SOURCE_DIR} && ${UNPACK_CMD} ../${TARBALL}
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
