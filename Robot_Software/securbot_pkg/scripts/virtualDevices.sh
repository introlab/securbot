#!/bin/bash

# Creates the virtual devices required securbot
# Kernel module manipulation requires root access.
# If lauched from ROS, ths script must be ran as root or using sudo through a user with the NOPASSWD setting

if [[ $EUID -ne 0 ]]; then
    sudo modprobe -r v4l2loopback
    sudo modprobe v4l2loopback video_nr=1,2 card_label="virtual_kinect","virtual_map"
else
    modprobe -r v4l2loopback
    modprobe v4l2loopback video_nr=1,2 card_label="virtual_kinect","virtual_map"
fi
