#!/bin/bash
#title           :network.sh
#description     :Set static IPv4 for ethernet and wifi
#author		     :Dominik Braaemer
#date            :15-03-2019
#version         :0.1
#==============================================================================

# Start wpa_supplicant with special config file in daemon mode
if [ $(pgrep wpa_supplicant) ]; then
    killall wpa_supplicant > /dev/null
    wpa_supplicant -i wlan0 -c /home/nao/wifi.config -u -D nl80211 -B
else
    wpa_supplicant -i wlan0 -c /home/nao/wifi.config -u -D nl80211 -B
fi

if [ ! -e /tmp/network.lock ]; then
    # Disable temporary ethernet and wifi
    ifconfig eth0 down
    ifconfig wlan0 down

    # Wait to let devices time to be disabled
    sleep 2s

    # Set ethernet IPv4 (IP must changed after copy this script to robot)
    ifconfig eth0 ROBOT_LAN_IP netmask 255.255.255.0

    # Set wifi IPv4 (IP must changed after copy this script to robot)
    ifconfig wlan0 ROBOT_WLAN_IP netmask 255.255.0.0

    # Enable ethernet and wifi
    ifconfig eth0 up
    ifconfig wlan0 up

    touch /tmp/network.lock
fi
