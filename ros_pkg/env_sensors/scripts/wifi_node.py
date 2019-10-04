#!/usr/bin/env python

import rospy
from pythonwifi.iwlibs import Wireless
from rtabmap_ros.msg import UserData
import struct

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

		# Create the User data
		dataMsg = UserData()
		dataMsg.header.frame_id = "Wifi_Level"
		dataMsg.header.stamp = stamp
		
		dataMsg.rows = 1
		dataMsg.cols = 1
		dataMsg.type = 6

		# Add data to the message
		dBm_ba = struct.pack('i',dBm)
		dataMsg.data = dBm_ba
		
		# Publish
		pub.publish(dataMsg)
		rate.sleep()


if __name__ == '__main__':
	try:
		wifiPub()
	except rospy.ROSInterruptException:
		pass
