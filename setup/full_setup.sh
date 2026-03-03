#!/bin/bash
set -e

# wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/full_setup.sh

echo "*** Install tools"
sudo apt install build-essential git cmake libboost-all-dev libairspyhf-dev libzmq3-dev libusb-1.0-0-dev pkg-config python3 python3-venv -y
git config --global pull.rebase false

echo "*** Create repos directory"
cd ~
if [ ! -d repos ]; then
    mkdir repos
fi
cd ~/repos

echo "*** Clone and build MavlinkTagController2 (controller + decimator + airspyhf_zeromq)"
cd ~/repos
if [ ! -d MavlinkTagController2 ]; then
	git clone --recurse-submodules git@github.com:DonLakeFlyer/MavlinkTagController2.git
fi
cd ~/repos/MavlinkTagController2
git pull origin main
git submodule update --init --recursive

echo "*** Build controller"
rm -rf build
make

echo "*** Set up Python virtual environment (detector + simulator)"
cd ~/repos/MavlinkTagController2
./setup_venv.sh
