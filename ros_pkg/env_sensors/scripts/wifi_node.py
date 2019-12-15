#!/usr/bin/env python

"""Provide node to publish current wifi level."""

import rospy
from pythonwifi.iwlibs import Wireless
from rtabmap_ros.msg import UserData
from env_sensors.msg import WifiSignal
import struct


def quality_to_dbm(quality):
    """Convert quality metric to dBm."""
    return quality / 2 - 100


def wifiPub():
    """Run a ROS node that published the current wifi level."""
    # Create node
    user_pub = rospy.Publisher('user_data_wifi', UserData, queue_size=1)
    wifi_pub = rospy.Publisher('wifi_level', WifiSignal, queue_size=1)
    rospy.init_node('wifi_pub', anonymous=False)
    rate = rospy.Rate(1)  # 1hz

    # setup wifi reading
    wifi = Wireless('wlan0')

    while not rospy.is_shutdown():
        # Get time stamp
        stamp = rospy.Time.now()

        # Find signal level
        quality = wifi.getQualityAvg()
        rssi = quality_to_dbm(quality.getSignallevel())
        noise = quality_to_dbm(quality.getNoiselevel())

        # Create the User data
        dataMsg = UserData()
        dataMsg.header.frame_id = "Wifi_Level"
        dataMsg.header.stamp = stamp

        dataMsg.rows = 1
        dataMsg.cols = 1
        dataMsg.type = 6

        # Add data to the message
        dBm_ba = struct.pack('i', rssi)
        dataMsg.data = dBm_ba

        # Create the wifi level
        wifiMsg = WifiSignal()
        wifiMsg.rssi = rssi
        wifiMsg.noise = noise

        # Publish
        user_pub.publish(dataMsg)
        wifi_pub.publish(wifiMsg)
        rate.sleep()


if __name__ == '__main__':
    try:
        wifiPub()
    except rospy.ROSInterruptException:
        pass
