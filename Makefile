
BOX2D_SRC=.box2d
BOX2D_CMAKE_OPTS=\
	-D BOX2D_SAMPLES=OFF \
	-D BOX2D_UNIT_TESTS=OFF

BOX2D_CMAKE_OPT_AVX=-D BOX2D_AVX

TARGET_LIN_AVX=box2d_linux_amd64_avx2.a

$(TARGET_LIN_AVX): submodules
	cd $(BOX2D_SRC) \
		&& cmake $(BOX2D_CMAKE_OPTS) $(BOX2D_CMAKE_OPT_AVX)=ON . \
		&& make \
		&& mv src/libbox2d.a ../binaries/box2d_linux_amd64_avx2.a

.PHONY: submodules
submodules:
	git submodule update --init --recursive --remote
