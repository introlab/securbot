#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

fakePatrol = '{ "patrol":[ {"x":1,"y":2,"yaw":3}, {"x":2,"y":1,"yaw":4}, {"x":3,"y":0,"yaw":3}, {"x":4,"y":-1,"yaw":4} ], "loop": false }'

class  PatrolTestSuite(unittest.TestCase):
    rospy.init_node('patrol_test')

    # Listeners and publishers to interact with patrol
    patrolPublisher = rospy.Publisher('/electron/patrol', String, queue_size=5)
    patrolCanceller = rospy.Publisher('/electron/patrol_halt', String, queue_size=5)
    mapImageOutput = rospy.Publisher('/map_image_generator/output_goal', PoseStamped, queue_size=20)

    conversionRequests = []
    move_base_actions = []

    def mapImageCallBack(self, data):
        self.conversionRequests.append(data)

    def moveBaseCallBack(self, data):
        self.move_base_actions.append(data)

    # Send json string and verify that waypoints are being queried for conversion
    def test_patrol_exec(self):
        rospy.Subscriber('/map_image_generator/input_goal', PoseStamped, self.mapImageCallBack)
        time.sleep(3)
        self.patrolPublisher.publish(fakePatrol)
        time.sleep(3)
        self.assertEquals(len(self.conversionRequests), 4, 'Move Base received :' + str(len(self.conversionRequests)) + ' elements')

    # Send conversion responses and make sure they got registered
    def test_waypoint_after_conversion(self):
        # Listen for movebase messages
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.moveBaseCallBack)
        time.sleep(3)

        # Send conveterted waypoints
        for waypoint in self.conversionRequests:
            self.mapImageOutput.publish(waypoint)
        time.sleep(3)

        # Assert messages being sent to movebase
        self.assertEquals(len(self.move_base_actions), 1, 'Expecting move_base to receive a goal (got: '+str(len(self.move_base_actions))+')')


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'patrol_exec', PatrolTestSuite)
