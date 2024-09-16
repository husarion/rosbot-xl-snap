#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

sudo snap connect rosbot-xl:raw-usb
sudo snap connect rosbot-xl:hardware-observe
sudo snap connect rosbot-xl:joystick
sudo snap connect rosbot-xl:shm-plug rosbot-xl:shm-slot
sudo snap connect rosbot-xl:shutdown

sudo rosbot-xl.restart