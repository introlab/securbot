#!/usr/bin/env python

import rospy
from pythonwifi.iwlibs import Wireless
from rtabmap_ros.msg import UserData
import numpy as np
import cv2

def wifiPub():
	# Create node
	pub = rospy.Publisher('wifi_level', UserData, queue_size=1)
	rospy.init_node('wifiPub',anonymous=False)
	rate = rospy.Rate(1) # 1hz
	
	# setup wifi reading
	wifi = Wireless('wlan0')

	while not rospy.is_shutdown():
		# Get time stamp
		stamp = rospy.Time.now()
		
		# Find signal level
		stats = wifi.getStatistics()
		dBm = stats[1].getSignallevel() - 256
		
		# Create maxtrix
		data = np.zeros((1,2), dtype = "uint8")
		data[0][0] = dBm
		data[0][1] = stamp.secs
		data2 = [np.uint8(stamp.secs),np.uint8(dBm)]
		
		# Create the User data
		dataMsg = UserData()
		dataMsg.header.frame_id = "Wifi_Level"
		dataMsg.header.stamp = stamp
		dataMsg.data = data2
		
		# Publish
		pub.publish(dataMsg)
		rate.sleep()


if __name__ == '__main__':
	try:
		wifiPub()
	except rospy.ROSInterruptException:
		pass
