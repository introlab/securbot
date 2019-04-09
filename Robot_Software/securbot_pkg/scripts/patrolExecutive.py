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

#GLOBAL VARIABLES
REAL_POSESTAMPED_INDEX = 3

#Global list of waypoints (2D with 3 columns) in different formats (Strings, Pixel PoseStampeds, Real PoseStampeds)
waypointsPatrolList = []

#Global publisher used to send Pixel PoseStampeds in order to format them into Real PoseStampeds
toMapImageGenerator = rospy.Publisher('/map_image_generator/goal', PoseStamped, queue_size=20)

#Global publisher to send waypoint is reached toward Electron node
toElectron = rospy.Publisher('toElectron', String, queue_size=20)

#Global publisher to send waypoints list to move_base
#TODO: Topic? Msg type?
toMoveBase = rospy.Publisher('', PoseStamped, queue_size=20)

#Format json Strings to Pixel PoseStamped
def jsonStringToPixelPoseStamped(jsonString):
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

#Publish Pixel PoseStamped to map_image_generator to format it into Real PoseStamped
#TODO: Be indepedant of map_image_generator
def pixelPoseStampedToRealPoseStamped(pixelPoseStamped)
  #Publishing to map_image_generator so this node can format into real PoseStamped                         
   toMapImageGenerator.publish(pixelPoseStamped) 
   
   rospy.loginfo("Pixel PoseStamped published to map_image_generator.")
   rospy.loginfo("Waiting Real PoseStamped from map_image_generator...")


#This receiver takes a Real PoseStamped publish by map_image_generator
def realPoseStampedReceiverCallback(realPoseStamped)
    rospy.loginfo("Received Real PoseStamped.")
    for waypoint in waypointsPatrolList
        if waypoint[REAL_POSESTAMPED_INDEX] == None:
            index = waypointsPatrolList.index(waypoint)
            waypointsPatrolList[index][REAL_POSESTAMPED_INDEX] = realPoseStamped
           
            #Checking if it was the last real PoseStamped to add to the list
            if (waypointsPatrolList.index(waypoint) + 1) == len(waypointsPatrolList)
               for waypoint in waypointsPatrolList 
                toMoveBase.publish(waypoint[REAL_POSESTAMP_INDEX]) 
            break

#This receiver takes a waypoints list (json as Strings) as a patrol planned for the robot and ensure every format needed for each waypoint are generated (Strings, Pixel PoseStampeds, Real PoseStampeds). It iterates through them gradually per waypoint reached. 
def waypointsListReceiverCallback(waypointsJsonStr):
    #Log Strings received before other formats generation
    rospy.loginfo(rospy.get_caller_id() + "Received json Strings waypoints :   %s   ", waypointsJsonStr.data)

    #Clear global patrol list for upcoming new list of waypoints
    waypointsPatrolList = []

    for waypointString in waypointsJsonStr.data
        #Format Json String to Pixel PoseStamped
        pixelPoseStamped = jsonStringToPixelPoseStamped(waypointString)

        #Fill global patrol list of waypoints with all formats generated (Except Real PoseStamped that as an asynchrous response, so force to None value)
        waypointsPatrolList.append([waypointString, pixelPoseStamped, None])

    #Loop that publish every Pixel PoseStamped to map_image_generator
    #This loop is after to ensure the partol list is ready to be iterate before waiting asynchrous response from map_image_generator
    for wp in waypointsPatrolList
        #Format Pixel PoseStamped to Real PoseStamped
        pixelPoseStampedToRealPoseStamped(wp[REAL_POSESTAMPED_INDEX])


def patrolExecutive():
    #Node name defined as patrolExecutive
    rospy.init_node('patrolExecutive', anonymous=True) #anonymous=True keeps each patrolExecutive nodes unique if there were many
    #Subscribing to topic '?' with callback
    #TODO: Topic? Msg type?
    rospy.Subscriber("", PoseStamped, realPoseStampedReceiverCallback)

    #Subscribing to topic 'fromElectronWaypoint' with callback
    #TODO: Topic?
    rospy.Subscriber("/operatorNavGoal", String, waypointsListReceiverCallback)

    #Subscribing to topic 'fromElectronInterrupt' with callback
    #TODO: Topic?
    rospy.Subscriber("/operatorNavGoal", String, interruptReceiverCallback)
    
    rospy.spin()

if __name__ == '__main__':
    patrolExecutive()
