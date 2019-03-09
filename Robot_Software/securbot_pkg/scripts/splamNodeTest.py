#!/usr/bin/env python

import rospy
from rtabmap_ros.srv import SetGoal


def handleSplamNodeTestServer(req):
    print "Receiving : %s"%(req)
    return req
def splamNodeTestServer():
    rospy.init_node('splamNodeTestServer')
    s = rospy.Service('splamNodeTestServer', SetGoal, handleSplamNodeTestServer)
    print "SPLAM NODE TEST SERVER READY"
    rospy.spin()

if __name__ == "__main__":
    splamNodeTestServer()





