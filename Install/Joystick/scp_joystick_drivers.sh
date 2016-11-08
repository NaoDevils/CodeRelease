#!/bin/bash
# Author: Heiner Walter
# Date: 26.11.2015
# 
# This script copies the drivers required for joystick control to the robot with the ip given as an argument.


# IP adress. Gets filled by arbument.
IP=$1
# Kernel version. Drivers are located in directory /lib/modules/KERNEL
# Warning: this must also be entered in line 33. The variable does not work there.
KERNEL="2.6.33.9-rt31-aldebaran-rt"
# Tell ssh where to find the key file.
KEY="-i ../../Config/Keys/id_rsa_nao"
# Path to directory (on this pc) where the kernel modules are stored.
# e.g. "../Files/kernel/2.6.33.9-rt31-aldebaran-rt"
MODULES_DIR="../Files/kernel/2.6.33.9-rt31-aldebaran-rt"
# Paths from $MODULES_DIR to the driver files.
DRIVERS_INPUT="kernel/drivers/input"
DRIVERS_JOYSTICK="$DRIVERS_INPUT/joystick"

if ! [ $# == 1 ]; then
  echo ""
  echo "This script copies the drivers required for joystick control to the robot with the ip given as an argument."
  echo "Usage: `basename $0` IP"
  echo "e.g.: ./`basename $0` 192.168.101.100"
  echo ""
  exit 0
fi

# Copy drivers to the robot.
scp $KEY $MODULES_DIR/$DRIVERS_INPUT/joydev.ko $MODULES_DIR/$DRIVERS_INPUT/ff-memless.ko $MODULES_DIR/$DRIVERS_INPUT/evdev.ko root@$IP:/lib/modules/$KERNEL/$DRIVERS_INPUT
ssh root@$IP $KEY $TEST 'mkdir /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/joystick'
scp $KEY $MODULES_DIR/$DRIVERS_JOYSTICK/xpad.ko root@$IP:/lib/modules/$KERNEL/$DRIVERS_JOYSTICK
# Search for new kernel modules.
ssh root@$IP $KEY 'depmod -A'

scp $KEY joystick_test root@$IP:/root
