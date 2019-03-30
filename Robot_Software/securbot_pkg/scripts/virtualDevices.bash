#!/bin/bash

# Creates the virtual devices required securbot
# Kernel module manipulation requires root access.
# If lauched from ROS, ths script must be ran as root or using sudo through a user with the NOPASSWD setting

if [[ $EUID -ne 0 ]]; then
    PREFIX="sudo"
else
    unset PREFIX
fi

$PREFIX modprobe -r v4l2loopback
$PREFIX modprobe v4l2loopback video_nr=1,2,3 card_label="virtual_kinect","virtual_map","fake_video"
