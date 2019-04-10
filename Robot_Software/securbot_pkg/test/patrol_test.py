#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

fakePatrol = '''{
    "patrol":[
        {"x":1,"y":2,"yaw":3},
        {"x":2,"y":1,"yaw":4},
        {"x":3,"y":0,"yaw":3},
        {"x":4,"y":-1,"yaw":4}
    ],
    "loop": false
}'''

class  PatrolTestSuite(unittest.TestCase):
    # Listeners and publishers to interact with patrol
    patrolPublisher = rospy.Publisher('/electron/patrol', String, queue_size=5)
    patrolCanceller = rospy.Publisher('/electron/patrol_halt', String, queue_size=5)
    mapImageOutput = rospy.Publisher('/map_image/output_goal', PoseStamped, queue_size=20)

    commandList = []
    def moveBaseCallBack(data):
        print(data)
        commandList.append(data)

    moveBaseSub = rospy.Subscriber('/move_base/goal', PoseStamped, moveBaseCallBack)


    def sendPatrol(self):
        patrolPublisher.publish(fakePatrol)
        time.sleep(2)
        print(commandList)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'patrol_exec', PatrolTestSuite)
