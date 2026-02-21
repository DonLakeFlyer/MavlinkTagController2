.PHONY: all controller decimator airspyhf_zeromq install test clean

# Default: build everything (parallel via CMake presets, jobs=0 = all cores)
all: controller decimator airspyhf_zeromq

controller:
	cmake --preset controller
	cmake --build --preset controller

decimator:
	cmake --preset decimator
	cmake --build --preset decimator

airspyhf_zeromq:
	cmake --preset airspyhf-zeromq
	cmake --build --preset airspyhf-zeromq

install: airspyhf_zeromq
	cmake --install build

test:
	cmake --preset debug
	cmake --build --preset debug
	ctest --preset debug

clean:
	@if [ -d build ]; then cmake --build build --target clean; else echo "No build/ directory"; fi
