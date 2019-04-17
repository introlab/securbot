#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time, actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction

FAKE_PATROL_1 = '{ "patrol":[ {"x":1,"y":2,"yaw":3}, {"x":2,"y":1,"yaw":4}, {"x":3,"y":0,"yaw":3}, {"x":4,"y":-1,"yaw":4} ], "loop": false }'

FAKE_PATROL_2 = '{ "patrol":[ {"x":1,"y":2,"yaw":3} ], "loop": false }'

TIMEOUT = 2 # seconds

INTERRUPT_TRUE_JSON = json.dumps({"interrupt":True})

class  PatrolTestSuite(unittest.TestCase):
    def __init__(self, *args):
        super(PatrolTestSuite, self).__init__(*args)
        rospy.init_node('patrol_test')
        # Move Base Mock
        self.actionServer = actionlib.SimpleActionServer(
                'move_base',
                MoveBaseAction,
                auto_start = False)
        self.actionServer.start()
        # Electron interface
        self.patrolPublisher = rospy.Publisher('/electron/patrol', String, queue_size = 5)
        self.patrolCanceller = rospy.Publisher('/electron/patrol_halt', String, queue_size = 5)
        rospy.Subscriber("/electron/patrol_feedback", String, self.waypointDoneCallback)
        # Map Image Mock
        self.conversionRequests = []

        # Waypoints Status Mock
        self.waypointsDoneStatus = []

        self.mapImageOutput = rospy.Publisher('/map_image_generator/output_goal',
                PoseStamped, queue_size = 20)
        rospy.Subscriber('/map_image_generator/input_goal', PoseStamped,
                self.mapImageCallBack)
        rospy.sleep(1) # waiting for subscribers and publishers to come online

    def mapImageCallBack(self, data):
        self.conversionRequests.append(data)

    def waypointDoneCallback(self, data):
        self.waypointsDoneStatus.append(json.loads(data)["waypointState"])

    # Setting up the test environment before every test
    def setUp(self):
        self.conversionRequests = []
        self.waypointsDoneStatus = []

    #
    # TEST FUNCTIONS
    #

    # Regular waypoint navigation
    def test_regular_patrol(self):
        self.patrolPublisher.publish(FAKE_PATROL_1)

        deadline = time.time() + TIMEOUT
        while not rospy.is_shutdown() and len(self.conversionRequests) < 4 and time.time() < deadline:
            rospy.sleep(0.1)

        self.assertEquals(len(self.conversionRequests), 4,
                'Move Base received :' + str(len(self.conversionRequests)) + ' elements')

        # Send converted waypoints
        for waypoint in self.conversionRequests:
            self.mapImageOutput.publish(waypoint)

        deadline = time.time() + TIMEOUT
        while not rospy.is_shutdown() and not self.actionServer.is_new_goal_available() and time.time() < deadline:
            rospy.sleep(0.1)

        # Assert messages being sent to movebase
        self.assertTrue(self.actionServer.is_new_goal_available(),
                'Expecting move_base to receive a goal')

    # Send waypoint and expect it is cancelled and send feedback to electron's node
    def test_waypoint_is_cancelled(self):

        time.sleep(3)

        # Publish fake patrol
        self.patrolPublisher.publish(FAKE_PATROL_2)
        time.sleep(3)

        # Publish interrupt
        self.patrolCanceller.publish(INTERRUPT_TRUE_JSON)
        time.sleep(3)

        # Assert waypoint have been interrupted
        self.assertTrue(len(self.waypointsDoneStatus) > 0)

        for status in self.waypointsDoneStatus:
            self.assertNotEqual(status, "PENDING")
            self.assertNotEqual(status, "ACTIVE")
            self.assertNotEqual(status, "SUCCEEDED")

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'patrol_exec', PatrolTestSuite)
