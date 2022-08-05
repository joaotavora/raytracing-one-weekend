CXX=clang++
CXXFLAGS=-gdwarf-4

configure-debug: export CXX      := ${CXX}
configure-debug: export CXXFLAGS := ${CXXFLAGS} -fsanitize=address -fsanitize=undefined
configure-debug:
	mkdir -p build-debug
	cd build-debug && conan install --build=missing ../
	cd build-debug && cmake -DCMAKE_BUILD_TYPE=Debug ../ -G"Unix Makefiles"

configure-release: export CXX      := ${CXX}
configure-release: export CXXFLAGS := ${CXXFLAGS}
configure-release:
	mkdir -p build-release
	cd build-release && conan install --build=missing ../
	cd build-release && cmake -DCMAKE_BUILD_TYPE=Release ../ -G"Unix Makefiles"

compile_commands.json:
	ln -sf build-release/compile_commands.json compile_commands.json

build-release:
	make -C build-release

build-debug:
	make -C build-debug

run: build-release
	./build-release/bin/rtweekend -q > /tmp/test.ppm

watch:
	feh -Z /tmp/test.ppm&
	rg --files src |                                                \
        entr -r -s 'make run'

bootstrap: configure-release build-release

clean:
	rm -rf build-debug build-release compile_commands.json

.PHONY: clean configure-debug configure-release run debug build-release build-debug
