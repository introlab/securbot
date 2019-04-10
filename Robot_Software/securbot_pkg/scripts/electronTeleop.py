#! /usr/bin/python

import rospy
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String

vel_msg = Twist()

publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def teleopCallback(msg):
        print(msg.data)
        j = json.loads(msg.data)
        print(j)
        # runCommand(j)

def teleopListener():
        rospy.init_node('electron_teleop')
        rospy.Subscriber('/fromElectron', String, teleopCallback)

        rospy.spin()

def runCommand(data):
        print(data)
        vel_msg.linear.x = data['x']
        vel_msg.linear.y = data['y']
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        publisher.publish(vel_msg)

        '''
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
        '''


if __name__ == '__main__':
        print('Ready for battle')
        teleopListener()
