# Setup Instructions

## Configure GitHub login for repo access

Setup auth for ssh connections.

```
sudo apt update
sudo apt install gh
gh auth login
```

## Linux/rPi setup

This will install/setup/build MavlinkController2, Airspy, csdr_uavrt, uavrt_detection, uavrt_channelize.

```
cd ~/Downloads
wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/full_setup.sh
sh full_setup.sh
```

### Setup rPi for UTC timezone

* To work around a problem with timezone difference between matlab code and the controll you must set the rPi timezone to UTC
* `sudo raspi-config`
* Select `Localization Options`
* Select `Timezone`
* Select `None of the above`
* Select `UTC`

### Setup rPi to start MavlinkTagController at boot

* Note that if your home directory is not `\home\pi` you will need to update the script
* run `crontab -e'
* Add this to the end of the file: `@reboot if [ -f /home/pi/MavlinkTagController.4.log ]; then mv -f /home/pi/MavlinkTagController.4.log /home/pi/MavlinkTagController.5.log; fi; if [ -f /home/pi/MavlinkTagController.3.log ]; then mv -f /home/pi/MavlinkTagController.3.log /home/pi/MavlinkTagController.4.log; fi; if [ -f /home/pi/MavlinkTagController.2.log ]; then mv -f /home/pi/MavlinkTagController.2.log /home/pi/MavlinkTagController.3.log; fi; if [ -f /home/pi/MavlinkTagController.1.log ]; then mv -f /home/pi/MavlinkTagController.1.log /home/pi/MavlinkTagController.2.log; fi; rm -f /home/pi/MavlinkTagController.log; /home/pi/repos/MavlinkTagController2/build/MavlinkTagController2 serial:///dev/ttyS0:57600 > /home/pi/MavlinkTagController.1.log 2>&1`

### Check on rPi whether controller is running

* `ps -aux | grep Mav`

## Testing with PX4 SITL

* Follow the instructions here to build/run a SITL version: 
  * `https://docs.px4.io/main/en/dev_setup/getting_started.html`
* Start MavlinkTagController: 
  * `cd ~/repos/MavlinkTagController2`
  * `./build/MavlinkTagControler2`