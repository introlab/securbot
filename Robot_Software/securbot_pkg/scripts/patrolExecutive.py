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
#Global indexes definitions
JSON_STRING_WAYPOINT_INDEX = 0
PIXEL_POSESTAMPED_INDEX = 1
REAL_POSESTAMPED_INDEX = 2
WAYPOINT_STATUS_INDEX = 3

#Global UINT8 waypoints statuses definitions
PENDING=0
ACTIVE=1
PREEMPTED=2
SUCCEEDED=3
ABORTED=4
REJECTED=5
PREEMPTING=6
RECALLING=7
RECALLED=8
LOST=9

#Global list of waypoints (2D with 4 columns) in different formats (Strings,
#Pixel PoseStampeds, Real PoseStampeds), plus their status
waypointsPatrolList = []

#Global active waypoint currently being processed by the action server
activeWaypoint = None

#Global indicating if the patrol received is looped
isLooped = False

#Global publisher used to send Pixel PoseStampeds in order to format them into
#Real PoseStampeds
toMapImageGenerator = rospy.Publisher("toMapImageGenerator", PoseStamped, queue_size=20)

#Global publisher to send status that waypoint is reached toward Electron node
toElectron = rospy.Publisher("toElectron", String, queue_size=20)

#Global publisher to send waypoints list to move_base
toMoveBase = rospy.Publisher("toMoveBase", PoseStamped, queue_size=20)

#Used instead into waypointsPatrolList
class Waypoint:
    def __init__(self, string, pixelPoseStamped, realPoseStamped, status):
        self.string = string
        self.pixelPoseStamped =pixelPoseStamped
        self.realPoseStamped = realPoseStamped
        self.status = status

#Format json Strings to Pixel PoseStamped
def jsonStringToPixelPoseStamped(jsonString):
    #Loading json message into buffer
    jsonBuffer = json.loads(jsonString)

    #Creating and partially filling a new PoseStamped
    waypoint = PoseStamped()
    waypoint.header.frame_id = "/map"
    waypoint.header.stamp = rospy.Time.now()

    #Formatting position
    waypoint.pose.position.x = jsonBuffer["x"]
    waypoint.pose.position.y = jsonBuffer["y"]
    waypoint.pose.position.z = 0

    #Formatting orientation
    roll = 0
    pitch = 0
    yaw = jsonBuffer["yaw"]
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    waypoint.pose.orientation.x = quaternion[0]
    waypoint.pose.orientation.y = quaternion[1]
    waypoint.pose.orientation.z = quaternion[2]
    waypoint.pose.orientation.w = quaternion[3]

    return waypoint

#Publish Pixel PoseStamped to map_image_generator to format it into Real
#PoseStamped
#TODO: Be indepedant of map_image_generator
def pixelPoseStampedToRealPoseStamped(pixelPoseStamped):
   toMapImageGenerator.publish(pixelPoseStamped)

   rospy.loginfo("Pixel PoseStamped published to map_image_generator.")
   rospy.loginfo("Waiting Real PoseStamped from map_image_generator...")


#This receiver takes a Real PoseStamped publish by map_image_generator
def realPoseStampedReceiverCallback(realPoseStamped):
    rospy.loginfo("Received Real PoseStamped.")
    for waypoint in waypointsPatrolList:
        if waypoint[REAL_POSESTAMPED_INDEX] == None:
            index = waypointsPatrolList.index(waypoint)
            waypointsPatrolList[index][REAL_POSESTAMPED_INDEX] = realPoseStamped

            #Checking if it was the last real PoseStamped to add to the list
            if (waypointsPatrolList.index(waypoint) + 1) == len(waypointsPatrolList):
               #Global variable iterating to help send the corresponding waypoint
               #reached
               activeWaypoint = waypointPatrolList[0]
               
               for waypoint in waypointsPatrolList: 
                toMoveBase.publish(waypoint[REAL_POSESTAMP_INDEX])
            break

#This receiver takes a waypoints list (json as Strings) as a patrol planned for
#the robot and ensure every format needed for each waypoint are generated
#(Strings, Pixel PoseStampeds, Real PoseStampeds). It iterates through them
#gradually per waypoint reached.
def waypointsListReceiverCallback(waypointsJsonStr):
    #Log Strings received before other formats generation
    rospy.loginfo(rospy.get_caller_id() + "Received json Strings waypoints :   %s   ", waypointsJsonStr.data)

    #Clear global patrol list for upcoming new list of waypoints and loop flag
    waypointsPatrolList = []
    isLooped = False

    waypointsStrings = json.loads(waypointsJsonStr.data)["patrol"]
    isLooped = json.loads(waypointsJsonStr.data)["loop"]

    for wpStr in waypointsStrings:
        #Format Json String to Pixel PoseStamped
        pixelPoseStamped = jsonStringToPixelPoseStamped(wpStr)

        #Fill global patrol list of waypoints with all formats generated
        #(Except Real PoseStamped that as an asynchrous response, so force to
        #None value)
        waypointsPatrolList.append([wpStr, pixelPoseStamped, None, None])#For now all waypoints status (column 4) are None values

    #Loop that publish every Pixel PoseStamped to map_image_generator
    #This loop is after to ensure the partol list is ready to be iterate before
    #waiting asynchrous response from map_image_generator
    for wp in waypointsPatrolList:
        #Format Pixel PoseStamped to Real PoseStamped
        pixelPoseStampedToRealPoseStamped(wp[PIXEL_POSESTAMPED_INDEX])

#Returns status as a string, used primarly for debugging purposes
def getStatusString(uInt8Status):
    if uInt8Status == PENDING:
        return "PENDING"
    elif uInt8Status == ACTIVE:
        return "ACTIVE"
    elif uInt8Status == PREEMPTED:
        return "PREEMPTED"
    elif uInt8Status == SUCCEEDED:
        return "SUCCEEDED"
    elif uInt8Status == ABORTED:
        return "ABORTED"
    elif uInt8Status == REJECTED:
        return "REJECTED"
    elif uInt8Status == PREEMPTING:
        return "PREEMPTING"
    elif uInt8Status == RECALLING:
        return "RECALLING"
    elif uInt8Status == RECALLED:
        return "RECALLED"
    elif uInt8Status == LOST:
        return "LOST"
    else:
        return "ERROR/UNKNOWN"
        
#TODO This receiver takes a PoseStamped
#def waypointsStatusReceiverCallback(waypointsStatus):
#    rospy.loginfo("Received waypoints status :" )
#    
#    nthWaypoint = 0
#    
#    for status in waypointsStatus.GoalStatus:
#        statusStr = getStatusString(status)
#        rospy.loginfo("Waypoint [%s] Status : [%s] ", nthWaypoint, statusStr)
#       if waypointsPatrolList[nthWaypoint][WAYPOINT_STATUS_INDEX] != statusStr :
#           
#           if status == ACTIVE && activeWaypoint != :
#               activeWaypoint = waypointsPatrolList[nthWaypoint]
#    TODO : do something if it's last waypoint reached and patrol's looped
    
def patrolExecutive():
    #Node name defined as patrolExecutive
    rospy.init_node("patrolExecutive", anonymous=True) #anonymous=True keeps each patrolExecutive nodes unique if there were many

    #Subscribing to topic 'fromMapImageGenerator' with callback
    rospy.Subscriber("fromMapImageGenerator", PoseStamped, realPoseStampedReceiverCallback)

    #Subscribing to topic 'fromElectronWaypoints' with callback
    rospy.Subscriber("fromElectronWaypoints", String, waypointsListReceiverCallback)
    

    #TODO Subscribing to topic 'fromElectronInterrupt' with callback
    #rospy.Subscriber("fromElectronInterrupt", String, interruptReceiverCallback)

    #TODO Subscribing to topic 'fromMoveBase' with callback
    #rospy.Subscriber("fromMoveBase", PoseStamped, 
    #waypointsStatusReceiverCallback)

    rospy.spin()

if __name__ == "__main__":
    patrolExecutive()
