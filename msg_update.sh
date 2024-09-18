#!/bin/bash
# Created by Nelson Durrant, Sep 2024

##########################################################
# RECOMPILES MICRO-ROS WITH UPDATED INTERFACES
# - Run this script after syncing changes using
#   'msg_sync.sh' in '~/ros2_ws/microros_tools'
##########################################################

cd ~/teensy_ws/agrobot
pio run --target clean_microros
pio lib install
pio run
