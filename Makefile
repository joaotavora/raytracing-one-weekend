CXX=clang++

all: release debug

build-debug: export CXXFLAGS := -gdwarf-4 -fsanitize=address -fsanitize=undefined
build-debug:   CMAKE_FLAGS=-DCMAKE_BUILD_TYPE=Debug   -G"Unix Makefiles"
build-release: CMAKE_FLAGS=-DCMAKE_BUILD_TYPE=Release -GNinja

build-%: export CXX := ${CXX}
build-%:
	mkdir -p build-$*
	cd build-$* && conan install --build=missing ../
	cd build-$* && cmake ${CMAKE_FLAGS} ../

compile_commands.json: build-debug
	ln -sf build-debug/compile_commands.json compile_commands.json

watch-%:
	feh -Z /tmp/test.ppm&
	rg --files src | entr -r -s 'make run-$*'

run-%:
	build-$*/bin/rtweekend > /tmp/test.ppm

release: build-release
	ninja -C build-release

debug: build-debug
	make -C build-debug

clean:
	rm -rf build-*

.PHONY: clean all release debug watch
