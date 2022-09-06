all: release debug

build-debug:   CXXFLAGS := -gdwarf-4 -fsanitize=address -fsanitize=undefined
build-debug:   CMAKE_FLAGS=-DCMAKE_BUILD_TYPE=Debug   -G"Unix Makefiles"

build-release: CMAKE_FLAGS=-DCMAKE_BUILD_TYPE=Release -G"Unix Makefiles"

build-%: export CXX := ${CXX}
build-%:
	mkdir -p build-$*
	(cd build-$* && conan install --build=missing --profile=${CXX} ../) \
              || (ret=$$?; rm -rf $@ && exit $$ret)
	(cd build-$* && CXXFLAGS='${CXXFLAGS}' cmake ${CMAKE_FLAGS} ../)       \
              || (ret=$$?; rm -rf $@ && exit $$ret)

watch-%:
	feh -Z /tmp/test.ppm&
	rg --files src | entr -r -s 'make run-$*'

run-%:
	build-$*/bin/rtweekend > /tmp/test.ppm

%: build-%
	make -C build-$*

clean:
	rm -rf build-*

.PHONY: clean all watch
