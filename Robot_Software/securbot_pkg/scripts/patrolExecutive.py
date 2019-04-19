#!/usr/bin/python

# Note definition: Waypoints is, by definition according to SecurBot, a goal to
#                 reach in the robot's physical environment

import rospy

# Json Strings formats (from web client) to PoseStamped for RTAB-Map
import json, hashlib, actionlib, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Yaws transformation into quaternions
from tf.transformations import quaternion_from_euler

# GLOBAL VARIABLES
# Global indexes definitions
JSON_STRING_WAYPOINT_INDEX = 0
PIXEL_POSESTAMPED_INDEX = 1
REAL_POSESTAMPED_INDEX = 2
WAYPOINT_STATUS_INDEX = 3

# Global UINT8 waypoints statuses definitions
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

# Global list of waypoints (2D with 4 columns) in different formats (Strings,
# Pixel PoseStampeds, Real PoseStampeds), plus their status
waypointsPatrolList = []
currentGoalIndex = 0

# Global indicating if the patrol received is looped
isLooped = False
patrolId = ""

# Global publisher used to send Pixel PoseStampeds in order to format them into
# Real PoseStampeds
toMapImageGenerator = rospy.Publisher("toMapImageGenerator", PoseStamped, queue_size=20)

# Global publisher to send status that waypoint is reached toward Electron node
toElectron = rospy.Publisher("toElectron", String, queue_size=20)

# TODO Used instead into waypointsPatrolList
class Waypoint:
    def __init__(self, string, pixelPoseStamped, realPoseStamped, status):
        self.string = string
        self.pixelPoseStamped =pixelPoseStamped
        self.realPoseStamped = realPoseStamped
        self.status = status

##  Format Waypoint to Pixel PoseStamped
#   @param waypointObject The waypoint in dictionnary form from the JSON object
#   @return PoseStamped representation of the waypoint
def wayPointToPixelPoseStamped(waypointObject):

    # Creating and partially filling a new PoseStamped
    waypoint = PoseStamped()
    waypoint.header.frame_id = "/map"
    waypoint.header.stamp = rospy.Time.now()

    # Formatting position
    waypoint.pose.position.x = waypointObject["x"]
    waypoint.pose.position.y = waypointObject["y"]
    waypoint.pose.position.z = 0

    # Formatting orientation
    roll = 0
    pitch = 0
    yaw = waypointObject["yaw"]
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    waypoint.pose.orientation.x = quaternion[0]
    waypoint.pose.orientation.y = quaternion[1]
    waypoint.pose.orientation.z = quaternion[2]
    waypoint.pose.orientation.w = quaternion[3]

    return waypoint

##  Publish Pixel PoseStamped to map_image_generator to format it into Real PoseStamped
#   @param pixelPoseStamped The waypoint relative to the screen input
def pixelPoseStampedToRealPoseStamped(pixelPoseStamped):
   toMapImageGenerator.publish(pixelPoseStamped)

   rospy.loginfo("Pixel PoseStamped published to map_image_generator.")
   rospy.loginfo("Waiting Real PoseStamped from map_image_generator...")


##  Callback funtion receiving processed waypoint from map_image
#   @param realPoseStamped The processed waypoint in dimesions relative to RTAB-MAP
def realPoseStampedReceiverCallback(realPoseStamped):
    rospy.loginfo("Received Real PoseStamped.")
    for waypoint in waypointsPatrolList:
        if waypoint[REAL_POSESTAMPED_INDEX] == None:
            index = waypointsPatrolList.index(waypoint)
            waypointsPatrolList[index][REAL_POSESTAMPED_INDEX] = realPoseStamped

            # Checking if it was the last real PoseStamped to add to the list
            if (waypointsPatrolList.index(waypoint) + 1) == len(waypointsPatrolList):
                startPatrolNavigation()
            break

##  This function starts sending the different waypoint that were converted in the list
def startPatrolNavigation():
    global currentGoalIndex, waypointsPatrolList, patrolId

    # Make sure no goals are active
    actionClient.cancel_all_goals()
    currentGoalIndex = 0

    rospy.loginfo("Starting navigation")
    publishPatrolFeedBack(patrolId, "start", currentGoalIndex, len(waypointsPatrolList))

    goal = MoveBaseGoal()
    goal.target_pose = waypointsPatrolList[currentGoalIndex][REAL_POSESTAMPED_INDEX]
    actionClient.send_goal(goal, sendGoalDoneCallback)


##  Callback funtion that handles the reception of new patrol plans.
#   This function will decode incoming JSON message and convert the waypoints
#   for them to be sent to move_base sequentially.
#   @param waypointsJsonStr The patrol plan in JSON representation
def waypointsListReceiverCallback(waypointsJsonStr):
    global waypointsPatrolList, isLooped, patrolId
    # Log Strings received before other formats generation
    rospy.loginfo(rospy.get_caller_id() + "Received json Strings waypoints :   %s   ", waypointsJsonStr.data)

    # Clear global patrol list for upcoming new list of waypoints and loop flag
    del waypointsPatrolList[:]
    isLooped = False

    try:
        waypointsJsonBuffer = json.loads(waypointsJsonStr.data)
    except TypeError:
        rospy.loginfo("ERROR :At json.loads buffer,TypeError, expected string or buffer!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    try:
         waypoints = waypointsJsonBuffer["patrol"]
    except KeyError:
        rospy.loginfo("ERROR : While accessing value at key [patrol] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    try:
        patrolId = waypointsJsonBuffer["id"]
    except KeyError:
        rospy.loginfo("ERROR : Missing patrol id, generating id")
        hasher = hashlib.sha1()
        hasher.update(waypointsJsonStr.data)
        patrolId = hasher.hexdigest()

    try:
         isLooped = waypointsJsonBuffer["loop"]
    except KeyError:
        rospy.loginfo("ERROR :While accessing value at key [loop] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Will assume no loops. Will patrol this list once.")

    for wpStr in waypoints:
        # Format waypoint to Pixel PoseStamped
        pixelPoseStamped = wayPointToPixelPoseStamped(wpStr)

        # Fill global patrol list of waypoints with all formats generated
        # (Except Real PoseStamped that as an asynchrous response, so force to
        # None value)
        waypointsPatrolList.append([wpStr, pixelPoseStamped, None, None])# For now all waypoints status (column 4) are None values


    # Loop that publish every Pixel PoseStamped to map_image_generator
    # This loop is after to ensure the partol list is ready to be iterate before
    # waiting asynchrous response from map_image_generator
    for wp in waypointsPatrolList:
        # Format Pixel PoseStamped to Real PoseStamped
        pixelPoseStampedToRealPoseStamped(wp[PIXEL_POSESTAMPED_INDEX])

##  Interprets interruption requests. Will stop patrol and send confirmation if
#   patrol plan is stopped.
#   @param interruptJsonStr The ROS message containing the JSON string with the interrupt message.
def interruptReceiverCallback(interruptJsonStr):
    # Log Strings received
    rospy.loginfo(rospy.get_caller_id() + "Received interrupt :   %s   ", interruptJsonStr.data)

    # Load json String
    isPatrolInterrupted = json.loads(interruptJsonStr.data)["interrupt"]

    assert isinstance(isPatrolInterrupted, bool)

    if isPatrolInterrupted:
        actionClient.cancel_all_goals()
        publishPatrolFeedBack(patrolId, "interrupt", currentGoalIndex, len(waypointsPatrolList))
        rospy.loginfo("Patrol interrupted.")
    else:
        rospy.loginfo("Patrol continuing. No interrupts received.")

##  Returns status as a string, used primarly for debugging purposes
#   @param uInt8Status move_base status code
#   @return String representation of the status
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

##  Publishes the advancement of a patrol
#   @param patrolId The unique id of the patrol in string representation
#   @param status String representation of the event to report
#   @param acheivedWaypointCount Number of waypoints that have succesfully been reached
#   @param plannedWaypointCount Number of waypoints in the current patrol
def publishPatrolFeedBack(patrolId, status, acheivedWaypointCount, plannedWaypointCount):
    assert isinstance(patrolId, str), "patrolId expected <type 'str'> and got "+str(type(patrolId))
    assert isinstance(status, str), "patrolId expected <type 'str'> and got "+str(type(status))
    assert isinstance(acheivedWaypointCount, int), "patrolId expected <type 'int'> and got "+str(type(acheivedWaypointCount))
    assert isinstance(plannedWaypointCount, int), "patrolId expected <type 'int'> and got "+str(type(plannedWaypointCount))
    feedback = {
                'patrolId': patrolId,
                'timestamp': time.time(),
                'status': status,
                'goalsReached': acheivedWaypointCount,
                'goalsPlanned': plannedWaypointCount
            }
    toElectron.publish(json.dumps(feedback))

##  Callback function called when an action sent to move_base is done
#   @param terminalState The state of the action (ie: SUCCEEDED / ABORTED)
#   @param result
def sendGoalDoneCallback(terminalState, result):
    global waypointsPatrolList, currentGoalIndex, patrolId

    rospy.loginfo("Received waypoint terminal state : [%s]", getStatusString(terminalState))

    # Check if goal was successfully achieved
    if terminalState != SUCCEEDED:
        # Stop patrol if goal could not be reached
        rospy.loginfo("Goal was not reached: terminating patrol")
        publishPatrolFeedBack(patrolId, "failed", currentGoalIndex, len(waypointsPatrolList))
        return
    else:
        currentGoalIndex += 1
        if currentGoalIndex >= len(waypointsPatrolList):
            publishPatrolFeedBack(patrolId, "finished", currentGoalIndex, len(waypointsPatrolList))
            rospy.loginfo("Patrol done. All waypoints reached.")
            if isLooped == True:
                rospy.loginfo("Restarting patrol with same waypoints...")
                startPatrolNavigation()
        else:
            publishPatrolFeedBack(patrolId, "waypoint_reached", currentGoalIndex, len(waypointsPatrolList))
            goal = MoveBaseGoal()
            goal.target_pose = waypointsPatrolList[currentGoalIndex][REAL_POSESTAMPED_INDEX]
            rospy.loginfo("Processing next waypoint...")
            actionClient.send_goal(goal, sendGoalDoneCallback)


## @fn patrolExecutive()
#  @brief Main function starting the Patrol Executive Node. Subcription to three topics are made : fromMapimageGenerator, fromElectronWaypoints, fromElectronInterrupt.
def patrolExecutive():
    # Node name defined as patrolExecutive
    rospy.init_node("patrolExecutive", anonymous=True) # anonymous=True keeps each patrolExecutive nodes unique if there were many

    # Subscribing to topic 'fromMapImageGenerator' with callback
    rospy.Subscriber("fromMapImageGenerator", PoseStamped, realPoseStampedReceiverCallback)

    # Subscribing to topic 'fromElectronWaypoints' with callback
    rospy.Subscriber("fromElectronWaypoints", String, waypointsListReceiverCallback)

    # Subscribing to topic 'fromElectronInterrupt' with callback
    rospy.Subscriber("fromElectronInterrupt", String, interruptReceiverCallback)

    rospy.spin()

if __name__ == "__main__":
    patrolExecutive()
