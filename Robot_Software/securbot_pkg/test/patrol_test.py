#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time, actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction

FAKE_PATROL = '{ "patrol":[ {"x":1,"y":2,"yaw":3}, {"x":2,"y":1,"yaw":4}, {"x":3,"y":0,"yaw":3}, {"x":4,"y":-1,"yaw":4} ], "loop": false }'
TIMEOUT = 2 # seconds

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
        self.patrolPublisher = rospy.Publisher('/electron/patrol', String, queue_size=5)
        self.patrolCanceller = rospy.Publisher('/electron/patrol_halt', String, queue_size=5)
        # Map Image Mock
        self.conversionRequests = []
        self.mapImageOutput = rospy.Publisher('/map_image_generator/output_goal',
                PoseStamped, queue_size=20)
        rospy.Subscriber('/map_image_generator/input_goal', PoseStamped,
                self.mapImageCallBack)

    def mapImageCallBack(self, data):
        self.conversionRequests.append(data)

    # Setting up the test environment before every test
    def setUp(self):
        self.conversionRequests = []


    #
    # TEST FUNCTIONS
    #

    # Regular waypoint navigation
    def test_regular_patrol(self):
        rospy.sleep(1) # waiting for subscribers and publishers to come online
        self.patrolPublisher.publish(FAKE_PATROL)

        deadline = time.time() + TIMEOUT
        while not rospy.is_shutdown() and len(self.conversionRequests) < 4 and time.time() < deadline:
            rospy.sleep(0.1)

        self.assertEquals(len(self.conversionRequests), 4,
                'Move Base received :' + str(len(self.conversionRequests)) + ' elements')

        # Send conveterted waypoints
        for waypoint in self.conversionRequests:
            self.mapImageOutput.publish(waypoint)

        deadline = time.time() + TIMEOUT
        while not rospy.is_shutdown() and not self.actionServer.is_new_goal_available() and time.time() < deadline:
            rospy.sleep(0.1)

        # Assert messages being sent to movebase
        self.assertTrue(self.actionServer.is_new_goal_available(),
                'Expecting move_base to receive a goal')


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'patrol_exec', PatrolTestSuite)
