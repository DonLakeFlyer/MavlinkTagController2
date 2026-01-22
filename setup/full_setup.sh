#!/bin/bash
set -e

# wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/full_setup.sh

echo "*** Install tools"
sudo apt install build-essential git cmake libboost-all-dev airspy libfftw3-dev -y
git config --global pull.rebase false

echo "*** Create repos directory"
cd ~
if [ ! -d repos ]; then
    mkdir repos
fi
cd ~/repos

echo "*** Clone and build MavlinkTagController"
cd ~/repos
if [ ! -d MavlinkTagController2 ]; then
	git clone --recursive https://github.com/DonLakeFlyer/MavlinkTagController2.git
fi
cd ~/repos/MavlinkTagController2
git pull origin main
make

echo "*** Clone and build csdr-uavrt"
cd ~/repos
if [ ! -d csdr-uavrt ]; then
	git clone --recursive https://github.com/DonLakeFlyer/csdr-uavrt.git
fi
cd ~/repos/csdr-uavrt
git pull origin master
make -j4
sudo make install

echo "*** Clone and build airspy_channelize"
cd ~/repos
if [ ! -d airspy_channelize ]; then
	git clone --recursive https://github.com/dynamic-and-active-systems-lab/airspy_channelize.git
fi
cd ~/repos/airspy_channelize
git pull origin main
make

echo "*** Clone and build uavrt_detection"
cd ~/repos
if [ ! -d uavrt_detection ]; then
	git clone --recursive https://github.com/dynamic-and-active-systems-lab/uavrt_detection.git
fi
cd ~/repos/uavrt_detection
git pull origin main
make
