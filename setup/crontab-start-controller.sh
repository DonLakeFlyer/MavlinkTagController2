#!/bin/bash
if [ -f /home/pi/MavlinkTagController.log ]; then
    mv -f /home/pi/MavlinkTagController.log /home/pi/MavlinkTagController.log.save
    tail -n 500 /home/pi/MavlinkTagController.log.save >> /home/pi/MavlinkTagController.log
    rm /home/pi/MavlinkTagController.log.save
fi
/home/pi/repos/MavlinkTagController2/build/controller/MavlinkTagController2 serial:///dev/serial0:921600 >>/home/pi/MavlinkTagController.log 2>&1
