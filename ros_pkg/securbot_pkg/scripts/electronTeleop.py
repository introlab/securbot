#! /usr/bin/python

## @package electronTeleop
#  This script subscribe to the electron node dealing the teleop data coming
#  from the UI, convert those datas and publish them on the cmd node of the base.
#
#  @author Cedric Godin <cedric.godin@usherbrooke.ca>
#  @author Edouard Legare <edouard.legare@usherbrooke.ca>

import rospy
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String

vel_msg = Twist()

publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

## Starts the ros node and subscribes to the electron node.
#
def teleopListener():
        rospy.init_node('electron_teleop', anonymous=True)
        rospy.Subscriber('fromElectron', String, teleopCallback)

        rospy.spin()

## Callback of the node subscription, parses the JSON string.
#  @param msg JSON string of the teleop datas.
#
def teleopCallback(msg):
        print(msg.data)
        j = json.loads(msg.data)
        print(j)
        runCommand(j)

## Sets the data in the Twist object and publishes them.
#  @param data teleop data.
#
def runCommand(data):
        print(data)
        vel_msg.linear.x = -data['y']
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = -data['x']

        publisher.publish(vel_msg)

## The package constructor
if __name__ == '__main__':
        print('Ready for battle')
        teleopListener()
