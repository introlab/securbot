#!/usr/bin/python

#Note definition: Waypoints is, by definition according to SecurBot, a goal to
#                 reach in the robot's physical environment

import rospy

#Json Strings formats (from web client) to PoseStamped for RTAB-Map
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

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


actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

#Global list of waypoints (2D with 4 columns) in different formats (Strings,
#Pixel PoseStampeds, Real PoseStampeds), plus their status
waypointsPatrolList = []

# Global waypoint iterator
waypointIterator = None

#Global indicating if the patrol received is looped
isLooped = False

#Global publisher used to send Pixel PoseStampeds in order to format them into
#Real PoseStampeds
toMapImageGenerator = rospy.Publisher("toMapImageGenerator", PoseStamped, queue_size=20)

#TODO Send feedback to UI with this publisher
#Global publisher to send status that waypoint is reached toward Electron node
toElectron = rospy.Publisher("toElectron", String, queue_size=20)

#TODO Used instead into waypointsPatrolList
class Waypoint:
    def __init__(self, string, pixelPoseStamped, realPoseStamped, status):
        self.string = string
        self.pixelPoseStamped =pixelPoseStamped
        self.realPoseStamped = realPoseStamped
        self.status = status

#Format Waypoint to Pixel PoseStamped
def wayPointToPixelPoseStamped(wayPointObject):

    #Creating and partially filling a new PoseStamped
    waypoint = PoseStamped()
    waypoint.header.frame_id = "/map"
    waypoint.header.stamp = rospy.Time.now()

    #Formatting position
    waypoint.pose.position.x = wayPointObject["x"]
    waypoint.pose.position.y = wayPointObject["y"]
    waypoint.pose.position.z = 0

    #Formatting orientation
    roll = 0
    pitch = 0
    yaw = wayPointObject["yaw"]
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
                startPatrolNavigation()
            break

# This function starts sending the different waypoint that were converted in the list
def startPatrolNavigation():
    global waypointIterator
    rospy.loginfo("Starting navigation")

    waypointIterator = iter(waypointsPatrolList)

    goal = MoveBaseGoal()
    goal.target_pose = next(waypointIterator)[REAL_POSESTAMPED_INDEX]
    actionClient.send_goal(goal, sendGoalDoneCallback)


#This receiver takes a waypoints list (json as Strings) as a patrol planned for
#the robot and ensure every format needed for each waypoint are generated
#(Strings, Pixel PoseStampeds, Real PoseStampeds). It iterates through them
#gradually per waypoint reached.
def waypointsListReceiverCallback(waypointsJsonStr):
    #Log Strings received before other formats generation
    rospy.loginfo(rospy.get_caller_id() + "Received json Strings waypoints :   %s   ", waypointsJsonStr.data)

    #Clear global patrol list for upcoming new list of waypoints and loop flag
    del waypointsPatrolList[:]
    isLooped = False

    try:
        waypointsJsonBuffer = json.loads(waypointsJsonStr.data)
    except TypeError:
        rospy.loginfo("ERROR :At json.loads buffer,TypeError, expected string or buffer!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    try:
         waypointsStrings = waypointsJsonBuffer["patrol"]
    except KeyError:
        rospy.loginfo("ERROR : While accessing value at key [patrol] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    try:
         isLooped = waypointsJsonBuffer["loop"]
    except KeyError:
        rospy.loginfo("ERROR :While accessing value at key [loop] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Will assume no loops. Will patrol this list once.")

    for wpStr in waypointsStrings:
        #Format waypoint to Pixel PoseStamped
        pixelPoseStamped = wayPointToPixelPoseStamped(wpStr)

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


def interruptReceiverCallback(interruptJsonStr):
    #Log Strings received
    rospy.loginfo(rospy.get_caller_id() + "Received interrupt :   %s   ", interruptJsonStr.data)

    #Load json String
    isPatrolInterrupted = json.loads(interruptJsonStr.data)["interrupt"]

    if isPatrolInterrupted:
        actionClient.cancel_all_goals()
        rospy.loginfo("Patrol interrupted.")
    else:
        rospy.loginfo("Patrol continuing. No interrupts received.")

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

#Change name for currentWaypointDoneCallback(terminalState, result)
def sendGoalDoneCallback(terminalState, result):
    global waypointIterator
    rospy.loginfo("Received waypoint terminal state : [%s]", getStatusString(terminalState))

    try:
        goal = MoveBaseGoal()
        goal.target_pose = next(waypointIterator)[REAL_POSESTAMPED_INDEX]
        rospy.loginfo("Processing next waypoint...")
        actionClient.send_goal(goal, sendGoalDoneCallback)
    except StopIteration:
        rospy.loginfo("Patrol done. All waypoints reached.")
        if isLooped == True:
            rospy.loginfo("Restarting patrol with same waypoints...")
            startPatrolNavigation()


def patrolExecutive():
    #Node name defined as patrolExecutive
    rospy.init_node("patrolExecutive", anonymous=True) #anonymous=True keeps each patrolExecutive nodes unique if there were many

    #Subscribing to topic 'fromMapImageGenerator' with callback
    rospy.Subscriber("fromMapImageGenerator", PoseStamped, realPoseStampedReceiverCallback)

    #Subscribing to topic 'fromElectronWaypoints' with callback
    rospy.Subscriber("fromElectronWaypoints", String, waypointsListReceiverCallback)

    #Subscribing to topic 'fromElectronInterrupt' with callback
    rospy.Subscriber("fromElectronInterrupt", String, interruptReceiverCallback)

    rospy.spin()

if __name__ == "__main__":
    patrolExecutive()
