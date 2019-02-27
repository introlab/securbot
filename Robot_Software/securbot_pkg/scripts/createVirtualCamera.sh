#!/bin/bash

if [ -z "$WEBRTC_CAM" ]; then
    sudo modprobe v4l2loopback video_nr=1 card_label="virtual_kinect"
    export WEBRTC_CAM=virtual_kinect
fi
