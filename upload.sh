#!/bin/bash
# Created by Nelson Durrant, Sep 2024

##########################################################
# UPLOADS HEX FILES TO THE TEENSY BOARD
# - Specify a file in firmware_options using 'bash 
#   upload.sh <file.hex>'
# - If this fails, check the USB connection
##########################################################

case $1 in
    "")
        cd ~/teensy_ws/agrobot/.pio/build/teensy41
        tycmd upload firmware.hex
        ;;
    *)
        cd ~/teensy_ws/firmware_options
        tycmd upload $1
        ;;
esac
