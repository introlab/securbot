#!/usr/bin/python

import rospy, json, hashlib, actionlib, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

# NOTE DEFINITION: Waypoints is, by definition according to 
#                  SecurBot, a goal to reach in the robot's physical
#                  environment.

# Enumerator of unsigned integers representating the status of a 
# waypoint the same way the action server from actionlib would 
# represent its statuses.
class Status(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9

# Global indexes used in the waypointsPatrolList to access the 
# PoseStampeds specific format
PIXEL_POSESTAMPED_INDEX = 1
REAL_POSESTAMPED_INDEX = 2


# Global action client used to sned waypoints to move_base
actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Global list of waypoints (2D with 4 columns) in different formats
# (Strings, Pixel PoseStampeds, Real PoseStampeds), plus their 
# status
waypointsPatrolList = []

# Global current waypoint index in the waypoints list
currentWaypointIndex = 0

# Global indicating if the patrol received is looped
isLooped = False

# Global patrol ID
patrolId = ""

# Global publisher used to send Pixel PoseStampeds in order to 
# format them into real PoseStampeds
toMapImageGenerator = rospy.Publisher("toMapImageGenerator", PoseStamped, queue_size=20)

# Global publisher to send status that waypoint is reached toward 
# Electron node
toElectron = rospy.Publisher("toElectron", String, queue_size=20)

##  Format Waypoint to Pixel PoseStamped
#   @param waypointObject The waypoint in dictionnary form from 
#          the JSON object
#   @return PoseStamped representation of the waypoint
def waypointToPixelPoseStamped(waypointObject):

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

##  Publish Pixel PoseStamped to map_image_generator to format it 
#   into Real PoseStamped
#   @param pixelPoseStamped The waypoint relative to the screen 
#                           input
def pixelPoseStampedToRealPoseStamped(pixelPoseStamped):
   toMapImageGenerator.publish(pixelPoseStamped)

   rospy.loginfo("Pixel PoseStamped published to map_image_generator.")
   rospy.loginfo("Waiting Real PoseStamped from map_image_generator...")


##  Callback funtion receiving processed waypoint from map_image
#   @param realPoseStamped The processed waypoint in dimesions 
#                          relative to RTAB-MAP
def realPoseStampedReceiverCallback(realPoseStamped):
    rospy.loginfo("Received Real PoseStamped.")
    for waypoint in waypointsPatrolList:
        if waypoint[REAL_POSESTAMPED_INDEX] == None:
            index = waypointsPatrolList.index(waypoint)
            waypointsPatrolList[index][REAL_POSESTAMPED_INDEX] = realPoseStamped

            # Checking if it was the last real PoseStamped to add 
            # to the list
            if (waypointsPatrolList.index(waypoint) + 1) == len(waypointsPatrolList):
                startPatrolNavigation()
            break

##  This function starts sending the different waypoint that were 
#   converted in the list
def startPatrolNavigation():
    global currentWaypointIndex, waypointsPatrolList, patrolId

    # Make sure no goals are active
    actionClient.cancel_all_goals()
    currentWaypointIndex = 0

    rospy.loginfo("Starting navigation")
    publishPatrolFeedBack(patrolId, "start", currentWaypointIndex, len(waypointsPatrolList))

    goal = MoveBaseGoal()
    goal.target_pose = waypointsPatrolList[currentWaypointIndex][REAL_POSESTAMPED_INDEX]
    actionClient.send_goal(goal, sendGoalDoneCallback)


##  Callback funtion that handles the reception of new patrol plans.
#   This function will decode incoming JSON message and convert the
#   waypoints for them to be sent to move_base sequentially.
#   @param waypointsJsonStr The patrol plan in JSON representation
def waypointsListReceiverCallback(waypointsJsonStr):
    global waypointsPatrolList, isLooped, patrolId
    # Log Strings received before other formats generation
    rospy.loginfo(rospy.get_caller_id() + "Received json Strings waypoints :   %s   ", waypointsJsonStr.data)

    # Clear global patrol list for upcoming new list of waypoints 
    # and loop flag
    del waypointsPatrolList[:]
    isLooped = False

    # Loads and ensure the data type is correct. Otherwise stop processing waypoints list.
    try:
        waypointsJsonBuffer = json.loads(waypointsJsonStr.data)
    except TypeError:
        rospy.loginfo("ERROR :At json.loads buffer,TypeError, expected string or buffer!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    # Buffer and ensure the key "patrol" is present. Otherwise stop processing waypoints list.
    try:
         waypoints = waypointsJsonBuffer["patrol"]
    except KeyError:
        rospy.loginfo("ERROR : While accessing value at key [patrol] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Ignoring waypoints list received...")
        return

    # Buffer and ensure the key "id" is present. Otherwise continue buffering json data.
    try:
        patrolId = waypointsJsonBuffer["id"]
    except KeyError:
        rospy.loginfo("ERROR : Missing patrol id, generating id")
        hasher = hashlib.sha1()
        hasher.update(waypointsJsonStr.data)
        patrolId = hasher.hexdigest()

    # Buffer and ensure the key "loop" is present. Otherwise assume no loops for this patrol.
    try:
         isLooped = waypointsJsonBuffer["loop"]
    except KeyError:
        rospy.loginfo("ERROR :While accessing value at key [loop] KeyError, non-existent or undefined in json string!")
        rospy.loginfo("Will assume no loops. Will patrol this list once.")

    for wpStr in waypoints:
        # Format waypoint to Pixel PoseStamped
        pixelPoseStamped = waypointToPixelPoseStamped(wpStr)

        # Fill global patrol list of waypoints with all formats 
        # generated
        # (Except Real PoseStamped that as an asynchrous response,
        # so force to None value)
        # For now all waypoints status (column 4) are None values
        waypointsPatrolList.append([wpStr, pixelPoseStamped, None, None])


    # Loop that publish every Pixel PoseStamped to 
    # map_image_generator
    # This loop is after to ensure the partol list is ready to be 
    # iterate before
    # waiting asynchrous response from map_image_generator
    for wp in waypointsPatrolList:
        # Format Pixel PoseStamped to Real PoseStamped
        pixelPoseStampedToRealPoseStamped(wp[PIXEL_POSESTAMPED_INDEX])

##  Interprets interruption requests. Will stop patrol and send 
#   confirmation if
#   patrol plan is stopped.
#   @param interruptJsonStr The ROS message containing the JSON 
#                           string with the interrupt message.
def interruptReceiverCallback(interruptJsonStr):
    # Log Strings received
    rospy.loginfo(rospy.get_caller_id() + "Received interrupt :   %s   ", interruptJsonStr.data)

    # Load json String
    isPatrolInterrupted = json.loads(interruptJsonStr.data)["interrupt"]

    assert isinstance(isPatrolInterrupted, bool)

    if isPatrolInterrupted:
        actionClient.cancel_all_goals()
        publishPatrolFeedBack(patrolId, "interrupt", currentWaypointIndex, len(waypointsPatrolList))
        rospy.loginfo("Patrol interrupted.")
    else:
        rospy.loginfo("Patrol continuing. No interrupts received.")

##  Returns status as a string, used primarly for debugging purposes
#   @param uInt8Status move_base status code
#   @return String representation of the status
def getStatusString(uInt8Status):
    if uInt8Status == Status.PENDING:
        return "PENDING"
    elif uInt8Status == Status.ACTIVE:
        return "ACTIVE"
    elif uInt8Status == Status.PREEMPTED:
        return "PREEMPTED"
    elif uInt8Status == Status.SUCCEEDED:
        return "SUCCEEDED"
    elif uInt8Status == Status.ABORTED:
        return "ABORTED"
    elif uInt8Status == Status.REJECTED:
        return "REJECTED"
    elif uInt8Status == Status.PREEMPTING:
        return "PREEMPTING"
    elif uInt8Status == Status.RECALLING:
        return "RECALLING"
    elif uInt8Status == Status.RECALLED:
        return "RECALLED"
    elif uInt8Status == Status.LOST:
        return "LOST"
    else:
        return "ERROR/UNKNOWN"

##  Publishes the advancement of a patrol
#   @param patrolId The unique id of the patrol in string 
#                   representation
#   @param status String representation of the event to report
#   @param acheivedWaypointCount Number of waypoints that have 
#                                succesfully been reached
#   @param plannedWaypointCount Number of waypoints in the current 
#                               patrol
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

##  Callback function called when an action sent to move_base is 
#   done
#   @param terminalState The state of the action 
#                        (ie: SUCCEEDED / ABORTED)
#   @param result
def sendGoalDoneCallback(terminalState, result):
    global waypointsPatrolList, currentWaypointIndex, patrolId

    rospy.loginfo("Received waypoint terminal state : [%s]", getStatusString(terminalState))

    # Check if goal was successfully achieved
    if terminalState != Status.SUCCEEDED:
        # Stop patrol if goal could not be reached
        rospy.loginfo("Goal was not reached: terminating patrol")
        publishPatrolFeedBack(patrolId, "failed", currentWaypointIndex, len(waypointsPatrolList))
        return
    else:
        currentWaypointIndex += 1
        # Check if all waypoints are done
        if currentWaypointIndex >= len(waypointsPatrolList):
            publishPatrolFeedBack(patrolId, "finished", currentWaypointIndex, len(waypointsPatrolList))
            rospy.loginfo("Patrol done. All waypoints reached.")
            # Check if the patrol has to be restarted
            if isLooped == True:
                rospy.loginfo("Restarting patrol with same waypoints...")
                startPatrolNavigation()
        # Proceed to next waypoint of the patrol
        else:
            publishPatrolFeedBack(patrolId, "waypoint_reached", currentWaypointIndex, len(waypointsPatrolList))
            goal = MoveBaseGoal()
            goal.target_pose = waypointsPatrolList[currentWaypointIndex][REAL_POSESTAMPED_INDEX]
            rospy.loginfo("Processing next waypoint...")
            actionClient.send_goal(goal, sendGoalDoneCallback)


##  Main function starting the Patrol Executive Node. Subcription to
#   three topics are made : fromMapimageGenerator, 
#                           fromElectronWaypoints, 
#                           fromElectronInterrupt.
def patrolExecutive():
    # Node name defined as patrolExecutive
    rospy.init_node("patrolExecutive", anonymous=True) 
    # anonymous=True keeps each patrolExecutive nodes unique if 
    # there were many

    # Subscribing to topic 'fromMapImageGenerator' with callback
    rospy.Subscriber("fromMapImageGenerator", PoseStamped, realPoseStampedReceiverCallback)

    # Subscribing to topic 'fromElectronWaypoints' with callback
    rospy.Subscriber("fromElectronWaypoints", String, waypointsListReceiverCallback)

    # Subscribing to topic 'fromElectronInterrupt' with callback
    rospy.Subscriber("fromElectronInterrupt", String, interruptReceiverCallback)

    rospy.spin()

if __name__ == "__main__":
    patrolExecutive()
