#!/bin/bash
#title           :sensorReader_init.sh
#description     :start sensorReader
#author		     :Dominik Brämer
#date            :04-03-2019
#version         :0.1
#==============================================================================

# Wait for shm of libbhuman, ndevilsbase
while true; do
    if [ -e /dev/shm/ndevils_mem ]; then
        break
    fi
    if [ -e /dev/shm/bhuman_mem ]; then
        break
    fi
    sleep 1s
done

/home/nao/bin/sensorReader
