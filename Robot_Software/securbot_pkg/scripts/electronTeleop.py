#! /usr/bin/python

import rospy
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String

vel_msg = Twist()

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def teleopListener():
        rospy.init_node('electron_teleop', anonymous=True)
        rospy.Subscriber('/fromElectron', String, teleopCallback)

        rospy.spin()

def teleopCallback(msg):
        print(msg.data)
        j = json.loads(msg.data)
        print(j)
        runCommand(j)

def runCommand(data):
        print(data)
        vel_msg.linear.x = -data['y']
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = -data['x']

        publisher.publish(vel_msg)


if __name__ == '__main__':
        print('Ready for battle')
        teleopListener()
