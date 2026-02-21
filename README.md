# Setup Instructions

## Related repositories

- AirspyHFDecimate: https://github.com/DonLakeFlyer/AirspyHFDecimate
- ZeroMQ IQ source (`airspyhf_zeromq_rx`): https://github.com/DonLakeFlyer/airspyhf-zeromq
- MAVLink controller (`MavlinkTagController2`): https://github.com/DonLakeFlyer/MavlinkTagController2

Protocol, packet-format, timing, and sample-rate assumption changes must be coordinated across all three repositories.

This repository also tracks the shared wire-format module as a submodule:

```
git submodule update --init --recursive
```

## Configure GitHub login for repo access

Setup auth for *https* connections.

```
sudo apt update
sudo apt install gh
gh auth login
```

## Linux/rPi setup

This will install/setup/build MavlinkController2, Airspy, csdr_uavrt, uavrt_detection, uavrt_channelize.

```
cd ~/Downloads
wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/full_setup.sh
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
* MAV_1_FORWARD: On
* SER_TEL2_BAUD: 921600 8N1
* Reboot Pixhawk

### Setup rPi to start MavlinkTagController at boot

* Note that if your home directory is not `/home/pi` you will need to update the reboot line
* run `crontab -e`
* Add this to the end of the file: `@reboot /bin/bash /home/pi/repos/MavlinkTagController2/setup/crontab-start-controller.sh`

### Check on rPi whether controller is running

* `ps -aux | grep Mav`

## Testing with PX4 SITL

* Follow the instructions here to build/run a SITL version:
  * `https://docs.px4.io/main/en/dev_setup/getting_started.html`
* Start MavlinkTagController:
  * `cd ~/repos/MavlinkTagController2`
  * `./build/MavlinkTagControler2`
