#!/bin/bash

function writeAliases() {
    echo "alias bhd='systemctl --user status bhumand'" >> /etc/bash/bashrc
    echo "alias ndd='systemctl --user status ndevild'" >> /etc/bash/bashrc
    echo "alias srd='systemctl --user status sensorReaderd'" >> /etc/bash/bashrc
    echo "" >> /etc/bash/bashrc
}

mount -o remount,rw /
umount -l /etc
if [ ! -e /etc/bash/bashrc.bak ]; then
    cp /etc/bash/bashrc /etc/bash/bashrc.bak
    writeAliases
else
    cp /etc/bash/bashrc.bak /etc/bash/bashrc
    writeAliases
fi
systemctl restart etc.mount