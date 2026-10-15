#!/bin/bash

# ExecStopPost of mavlink-tag-controller.service. The controller is exec'd, so no
# shell is left to print "Segmentation fault"; record systemd's view of the exit instead.
echo "[$(date -u +%H:%M:%S)|E]  Controller exited: result=${SERVICE_RESULT:-?} code=${EXIT_CODE:-?} status=${EXIT_STATUS:-?}" >>/home/pi/MavlinkTagController.log
