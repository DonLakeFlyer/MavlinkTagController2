# Instructions for Linux(SITL)/rPi vehicle setup

```
cd ~
mkdir repos
cd ~/repos
```

### Boost Library
```
sudo apt-get install libboost-all-dev
```
### Install CMake and others
```
sudo apt update
sudo apt-get install libssl-dev
sudo apt install build-essential libtool autoconf unzip wget
```
 Go to the official [CMake webpage](https://www.cmake.org/download), then download and extract the latest version. Update the version and build variables in the following command to get the desired version:
 ```
version=3.29
build=1
## don't modify from here
mkdir ~/temp
cd ~/temp
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
tar -xzvf cmake-$version.$build.tar.gz
cd cmake-$version.$build/
```
 Install the extracted source by running:
 ```
 ./bootstrap
make -j$(nproc)
sudo make install
```
Test your new cmake version.
```
$ cmake --version
```

### MavlinkTagController
This is the controller on the vehicle which talks back/forth with QGC.

```
cd ~/repos
git clone --recursive git@github.com:DonLakeFlyer/MavlinkTagController2.git
cd MavlinkTagController2
make
```


### Airspy
`sudo apt install airspy`

### libfftw30-dev
Needed by csdr
```
sudo apt-get update
sudo apt-get install libfftw3-dev
```


### csdr_uavrt
This is a custom version of csdr which outputs to udp

```
cd ~/repos
git clone git@github.com:DonLakeFlyer/csdr-uavrt.git
cd csdr-uavrt 
make
sudo make install
```

### uavrt_detection
```
cd ~/repos
git clone --recursive git@github.com:dynamic-and-active-systems-lab/uavrt_detection.git
cd uavrt_detection 
make
```

### airspy_channelize

#### AirSpy Mini version
```
cd ~/repos
git clone --recursive git@github.com:dynamic-and-active-systems-lab/airspy_channelize.git airspy_channelize_mini
cd airspy_channelize_mini
make
```

#### AirSpy HF version
```
cd ~/repos
git clone --recursive -b airspy-hf git@github.com:dynamic-and-active-systems-lab/airspy_channelize.git airspy_channelize_hf
cd airspy_channelize_hf
make
```

## Tag Tracker QGC
You can get an insaller artifact from here: `https://github.com/DonLakeFlyer/UAV-RT-TagTracker/actions`

You need libfuse2 to run the appimages produced by GitHub. Install instructions here: https://github.com/AppImage/AppImageKit/wiki/FUSE
On Ubuntu (>= 22.04):
```
sudo add-apt-repository universe
sudo apt install libfuse2
```
Install libxcb-xinerama0
```
sudo apt install libxcb-xinerama0
```

Make executable and run program
```
chmod +x TagTracker.AppImage
./TagTracker.AppImage
```

## Testing with PX4 SITL
* Follow the instructions here to build a SITL version: https://docs.px4.io/main/en/dev_setup/dev_env.html.
  * https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
  * `cd ~/repos`
  * `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`
  * `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`
  * Install kconfiglib: `pip3 install kconfiglib`
* Start PX4 SITL: `PX4_SIM_SPEED_FACTOR=2 HEADLESS=1 make -j 12 px4_sitl_default gazebo-classic_iris`
  * You may need to use jmavsim rather than gazerbo-classic_iris
* Start MavlinkTagController: 
  * `cd ~/repos/MavlinkTagController2`
  * `./build/MavlinkTagControler2`
* Start QGC Tag Tracker version
  * Click Tags to send tags
  * Click Start to start detectors
  * Click Stop to stop detectors
  * Update the TagInfo.txt to match you tag
  * Application Settings/Tags to set K, False Alarm and so forth

## Setup rPi for UTC timezone

* To work around a problem with timezone difference between matlab code and the controll you must set the rPi timezone to UTC
* `sudo raspi-config`
* Select `Localization Options`
* Select `Timezone`
* Select `None of the above`
* Select `UTC`

## Setup rPi to start MavlinkTagController at boot

* Note that if your home directory is not `\home\pi` you will need to update the script
* run `crontab -e'
* Add this to the end of the file: `@reboot if [ -f /home/pi/MavlinkTagController.4.log ]; then mv -f /home/pi/MavlinkTagController.4.log /home/pi/MavlinkTagController.5.log; fi; if [ -f /home/pi/MavlinkTagController.3.log ]; then mv -f /home/pi/MavlinkTagController.3.log /home/pi/MavlinkTagController.4.log; fi; if [ -f /home/pi/MavlinkTagController.2.log ]; then mv -f /home/pi/MavlinkTagController.2.log /home/pi/MavlinkTagController.3.log; fi; if [ -f /home/pi/MavlinkTagController.1.log ]; then mv -f /home/pi/MavlinkTagController.1.log /home/pi/MavlinkTagController.2.log; fi; rm -f /home/pi/MavlinkTagController.log; /home/pi/repos/MavlinkTagController2/build/MavlinkTagController2 serial:///dev/ttyS0:57600 > /home/pi/MavlinkTagController.1.log 2>&1`

## Check on rPi whether controller is running

* `ps -aux | grep Mav`
