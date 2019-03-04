#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

vel_msg = Twist()

def commandCallback(msg):
	print(msg.data)
	runCommand(msg.data)

def runCommand(data):
	print(data)
	if data == 'forward':
		vel_msg.linear.x = 0.2
		vel_msg.linear.y = 0.0
		vel_msg.linear.z = 0.0
		vel_msg.angular.x = 0.0
		vel_msg.angular.y = 0.0
		vel_msg.angular.z = 0.0
	elif data == 'left':
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.8
	elif data == 'right':
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = -0.8
	elif data == 'backward':
		vel_msg.linear.x = -0.2
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0
	else:
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0

publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
subscriber = rospy.Subscriber('/fromElectron', String, commandCallback)

rospy.init_node('electron_teleop')
rate = rospy.Rate(10)

print('Ready for battle')
while not rospy.is_shutdown():
	publisher.publish(vel_msg)
	rate.sleep()
