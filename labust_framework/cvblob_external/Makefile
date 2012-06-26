all: cvblob
TARBALL = build/cvblob-0.10.4-src.tgz
TARBALL_URL = http://cvblob.googlecode.com/files/cvblob-0.10.4-src.tgz
SOURCE_DIR = build/cvblob
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

cvblob: ${SOURCE_DIR}/unpacked
	cd $(SOURCE_DIR) && cmake . && make
	#mkdir -p lib
	#mkdir -p include/cvBlob
	#cp -rf ${SOURCE_DIR}/cvBlob/*.h include/cvBlob/
	#cp -rf ${SOURCE_DIR}/lib/*.so lib/
	#rm -rf ${SOURCE_DIR}
clean:
	-rm -rf src $(SOURCE_DIR) paramiko
wipe: clean
	-rm -rf build
