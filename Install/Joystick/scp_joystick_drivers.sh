#!/bin/bash
# Author: Heiner Walter
# Author: Sebastian Hoose
# Date: 05.06.2019
# 
# This script copies the drivers required for joystick control to the robot with the ip given as an argument.


# IP adress. Gets filled by argument.
IP=$1
# Robot Version. Gets filled by argument.
VERSION=$2

if ! [ $# == 2 ]; then
  echo ""
  echo "This script copies the drivers required for joystick control to the robot with the ip and robot version given as an argument."
  echo "Usage: `basename $0` IP VERSION"
  echo "e.g.: ./`basename $0` 192.168.101.100 V5"
  echo ""
  exit 0
fi

 # Choose kernel version. Drivers are located in directory /lib/modules/KERNEL
if [ $VERSION = "V5" ]; then
    KERNEL="2.6.33.9-rt31-aldebaran-rt"
    MODULES_DIR="../Files/kernel/2.6.33.9-rt31-aldebaran-rt"
else
    if [ $VERSION = "V6" ]; then
        KERNEL="4.4.86-rt99-aldebaran"
        MODULES_DIR="../Files/kernel/4.4.86-rt99-aldebaran"
    else
        echo "Unknown robot version. Choose V5 or V6"
    fi
fi

# Tell ssh where to find the key file.
KEY="-i ../../Config/Keys/id_rsa_nao"

# Paths from $MODULES_DIR to the driver files.
DRIVERS_INPUT="kernel/drivers/input"
DRIVERS_JOYSTICK="$DRIVERS_INPUT/joystick"

# deploy and activate drivers
if [ $VERSION = "V5" ]; then
    # Copy drivers to the robot.
    scp $KEY $MODULES_DIR/$DRIVERS_INPUT/joydev.ko $MODULES_DIR/$DRIVERS_INPUT/ff-memless.ko $MODULES_DIR/$DRIVERS_INPUT/evdev.ko root@$IP:/lib/modules/$KERNEL/$DRIVERS_INPUT
    ssh root@$IP $KEY $TEST 'mkdir /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/joystick'
    scp $KEY $MODULES_DIR/$DRIVERS_JOYSTICK/xpad.ko root@$IP:/lib/modules/$KERNEL/$DRIVERS_JOYSTICK
    # Search for new kernel modules.
    #ssh root@$IP $KEY 'depmod -A' # This causes the audio driver to fail
    # Load joystick drivers manually
    ssh root@$IP $KEY 'insmod /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/joydev.ko'
    ssh root@$IP $KEY 'insmod /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/ff-memless.ko'
    ssh root@$IP $KEY 'insmod /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/evdev.ko'
    ssh root@$IP $KEY 'insmod /lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/joystick/xpad.ko'
#    scp $KEY joystick_test root@$IP:/root
else
    if [ $VERSION = "V6" ]; then
        scp $KEY $MODULES_DIR/$DRIVERS_INPUT/joydev.ko $MODULES_DIR/$DRIVERS_INPUT/ff-memless.ko $MODULES_DIR/$DRIVERS_INPUT/evdev.ko $MODULES_DIR/$DRIVERS_JOYSTICK/xpad.ko nao@$IP:~/
        echo "The password you are asked for is \"nao\""
        ssh -t nao@$IP $KEY "sudo mount -o remount,rw '/' && sudo mkdir /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/joystick && sudo mv /home/nao/joydev.ko /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input && sudo mv /home/nao/ff-memless.ko /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input && sudo mv /home/nao/evdev.ko /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input && sudo mv /home/nao/xpad.ko /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/joystick && sudo depmod && sudo insmod /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/joydev.ko && sudo insmod /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/ff-memless.ko && sudo insmod /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/evdev.ko && sudo insmod /lib/modules/4.4.86-rt99-aldebaran/kernel/drivers/input/joystick/xpad.ko"
    fi
fi

