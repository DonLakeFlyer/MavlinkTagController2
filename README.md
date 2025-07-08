# Setup Instructions

## Configure GitHub login for repo access

Setup auth for *ssh* connections.

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

### Setup rPi for UTC timezone and Serial Port usage

To work around a problem with timezone difference between matlab code and the controll you must set the rPi timezone to UTC:
* `sudo raspi-config`
* Select `Localization Options`
* Select `Timezone`
* Select `None of the above`
* Select `UTC`

Enable serial port usage by FMU:
* `sudo raspi-config`
* Select `Interface Options`
* Select `Serial Port`
* Answer `No` to `Login shell accessible over serial`
* Answer `Yes` to `Serial port hardware enabled`

### Setup Pixhawk to talk to rPi CM4 over serial

Adjust these parameters:
* MAV_1_CONFIG: TELEM2
* MAV_1_MODE: Onboard
* SER_TEL2_BAUD: 921600 8N1
* Reboot Pixhawk

### Setup rPi to start MavlinkTagController at boot

* Note that if your home directory is not `\home\pi` you will need to update the script
* run `crontab -e'
* Add this to the end of the file: `@reboot if [ -f /home/pi/MavlinkTagController.log ]; then mv -f /home/pi/MavlinkTagController.log /home/pi/MavlinkTagController.log.save; tail -n 500 /home/pi/MavlinkTagController.log.save >> /home/pi/MavlinkTagController.log; rm /home/pi/MavlinkTagController.log.save; fi; /home/pi/repos/MavlinkTagController2/build/MavlinkTagController2 serial:///dev/serial0:921600 2>&1 >>/home/pi/MavlinkTagController.log`

### Check on rPi whether controller is running

* `ps -aux | grep Mav`

## Testing with PX4 SITL

* Follow the instructions here to build/run a SITL version: 
  * `https://docs.px4.io/main/en/dev_setup/getting_started.html`
* Start MavlinkTagController: 
  * `cd ~/repos/MavlinkTagController2`
  * `./build/MavlinkTagControler2`
