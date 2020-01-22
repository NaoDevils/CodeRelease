#!/bin/bash

export LFS=/data/chrt
umount -l $LFS/dev/pts
umount -l $LFS/dev
umount -l $LFS/tmp
umount -l $LFS/home
umount -l $LFS/usr/share/alsa
umount -l $LFS/usr/lib/pulse-9.0 
umount -l $LFS/usr/lib/pulseaudio 
umount -l $LFS/usr/lib/i386-linux-gnu/alsa-lib
umount -l $LFS/opt/ros/indigo/lib
umount -l $LFS/etc/pulse
umount -l $LFS/proc
umount -l $LFS/sys
umount -l $LFS/run
umount -l $LFS/var/run
