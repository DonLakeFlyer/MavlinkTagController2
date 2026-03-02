.PHONY: all controller decimator airspyhf_zeromq install test clean

# Default: build everything (parallel via CMake presets, jobs=0 = all cores)
all: controller decimator airspyhf_zeromq

controller:
	cmake --preset controller-release
	cmake --build --preset controller-release

decimator:
	cmake --preset decimator-release
	cmake --build --preset decimator-release

airspyhf_zeromq:
	cmake --preset airspyhf-zeromq-release
	cmake --build --preset airspyhf-zeromq-release

install: airspyhf_zeromq
	cmake --install build

test:
	cmake --preset release
	cmake --build --preset release
	ctest --preset release

clean:
	@if [ -d build ]; then cmake --build build --target clean; else echo "No build/ directory"; fi
