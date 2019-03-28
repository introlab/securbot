#TODO: Try to understand how to not have this line below without having an error
#coding=utf-8

#!/usr/bin/python

"""
Copyright (c) 2019, Dominic Létourneau - IntRoLab - Universite de Sherbrooke,
SecurBot - Universite de Sherbrooke, All rights reserved. Redistribution and
use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
* Neither the name of the Universite de Sherbrooke nor the names of its
  contributors may be used to endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

#Note definition: Waypoints is, by definition according to SecurBot, a goal to
#                 reach in the robot's physical environment

import rospy

#Json Strings formats (from web client) to PoseStamped for RTAB-Map
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

#Yaws transformation into quaternions
from tf.transformations import quaternion_from_euler

#Global publisher that can be used
waypointPublisher = rospy.Publisher('/map_image_generator/goal', PoseStamped, queue_size=10)


#Formatter function from json string to PoseStamped
def jsonStringToPoseStamped(jsonString):
    rospy.loginfo("Formatting JSON waypoint string to PoseStamped...")
    rospy.loginfo("JSON to format : "+ jsonString)
    
    #Loading json message into buffer
    jsonBuffer = json.loads(jsonString)
    
    #Creating and partially filling a new PoseStamped
    waypoint = PoseStamped()
    waypoint.header.frame_id = "/map"
    waypoint.header.stamp = rospy.Time.now()

    #Formatting position
    waypoint.pose.position.x = jsonBuffer['x']
    waypoint.pose.position.y = jsonBuffer['y']
    waypoint.pose.position.z = 0

    #Formatting orientation
    roll = 0
    pitch = 0
    yaw = jsonBuffer['yaw']
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    waypoint.pose.orientation.x = quaternion[0]
    waypoint.pose.orientation.y = quaternion[1]
    waypoint.pose.orientation.z = quaternion[2]
    waypoint.pose.orientation.w = quaternion[3]

    rospy.loginfo("Waypoint : ")
    rospy.loginfo("")
    rospy.loginfo(waypoint)

    return waypoint

#Publishing waypoint string message formatted as json toward '/map_image_generator/goal' topic
def waypointToMapImageGeneratorCallback(waypointJsonStr):
    rospy.loginfo(rospy.get_caller_id() + " heard   %s   ", waypointJsonStr.data)

    #Format Json String to PoseStamped
    newPoseStamped = jsonStringToPoseStamped(waypointJsonStr.data)

    #Publishing
    waypointPublisher.publish(newPoseStamped)

def waypointListener():
    #Node name defined as waypointDecoder
    rospy.init_node('waypointDecoder', anonymous=True) #anonymous=True keeps each waypointDecoder nodes unique if there were many
    #Subscribing to topic 'fromElectron' with callback
    rospy.Subscriber("fromElectron", String, waypointToMapImageGeneratorCallback)
    rospy.spin()

if __name__ == '__main__':
    waypointListener()
