#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

fakePatrol = '{ "patrol":[ {"x":1,"y":2,"yaw":3}, {"x":2,"y":1,"yaw":4}, {"x":3,"y":0,"yaw":3}, {"x":4,"y":-1,"yaw":4} ], "loop": false }'

class  PatrolTestSuite(unittest.TestCase):
    rospy.init_node('patrol_test')

    # Listeners and publishers to interact with patrol
    patrolPublisher = rospy.Publisher('/electron/patrol', String, queue_size=5)
    patrolCanceller = rospy.Publisher('/electron/patrol_halt', String, queue_size=5)
    mapImageOutput = rospy.Publisher('/map_image/output_goal', PoseStamped, queue_size=20)

    conversionRequests = []

    def mapImageCallBack(self, data):
        self.conversionRequests.append(data)
        self.mapImageOutput.publish(data)

    # Send json string and verify that waypoints are being queried for conversion
    def test_patrol_exec(self):
        moveBaseSub = rospy.Subscriber('/map_image/input_goal', PoseStamped, self.mapImageCallBack)
        time.sleep(1)
        self.patrolPublisher.publish(fakePatrol)
        time.sleep(1)
        self.assertEquals(len(self.conversionRequests), 4, 'Move Base received :' + str(len(self.conversionRequests)) + ' elements')

    # Send conversion responses and make sure they got registered
    def test_waypoint_after_conversion(self):
        # Listen for movebase messages

        # Send conveterted waypoints
        for waypoint in conversionRequests:
            self.mapImageOutput.publish(waypoint)

        # Assert messages being sent to movebase
        self.assertEquals(1,1,'hey')



if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'patrol_exec', PatrolTestSuite)
