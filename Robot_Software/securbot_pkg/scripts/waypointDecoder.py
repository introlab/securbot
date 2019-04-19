#!/usr/bin/python

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
    rospy.Subscriber("/operatorNavGoal", String, waypointToMapImageGeneratorCallback)
    rospy.spin()

if __name__ == '__main__':
    waypointListener()
