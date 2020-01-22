#!/bin/bash
#title           :chroot_init.sh
#description     :Initialize chroot, start ndevilsbase and start bhuman
#author		     :Dominik Brämer
#date            :18-02-2019
#version         :0.1
#==============================================================================

# Wait for lola socket
while true; do
    if [ -e /tmp/robocup ]; then
        break
    fi
    sleep 1s
done

/usr/libexec/reset-cameras.sh toggle

sleep 2s

/home/nao/bin/bhuman -w
