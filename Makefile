
BOX2D_SRC=.box2d
BUILD_DIR=build

BOX2D_CMAKE_OPTS=\
	-DBOX2D_SAMPLES=OFF \
	-DBOX2D_BUILD_DOCS=OFF \
	-DBOX2D_UNIT_TESTS=OFF
BOX2D_CMAKE_OPT_AVX=\
	-DBOX2D_AVX

TARGET_LIN_AVX=box2d_linux_amd64_avx2.a

$(TARGET_LIN_AVX): submodules
	cd $(BOX2D_SRC) \
		&& rm -rf $(BUILD_DIR) \
		&& mkdir $(BUILD_DIR) \
		&& cd $(BUILD_DIR) \
		&& cmake $(BOX2D_CMAKE_OPTS) $(BOX2D_CMAKE_OPT_AVX)=ON .. \
		&& cmake --build . \
		&& mv src/libbox2d.a ../../binaries/$(TARGET_LIN_AVX)

.PHONY: submodules
submodules:
	git submodule update --init --recursive --remote
