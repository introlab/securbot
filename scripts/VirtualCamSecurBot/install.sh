#!/bin/bash
echo "Staring Installation..."
sudo apt update && sudo apt install linux-generic v4l2loopback-dkms ffmpeg -y
echo "Installation finished..."
