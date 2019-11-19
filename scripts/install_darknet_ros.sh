#!/bin/bash

#See github repository : https://github.com/leggedrobotics/darknet_ros

#Installs darknet_ros in catkin_ws

#Dependencies: -OpenCV
#              -boost
#              -Git

echo Installing DARKNET ROS...
cd ~/catkin_ws/src/
git clone https://github.com/leggedrobotics/darknet_ros.git

