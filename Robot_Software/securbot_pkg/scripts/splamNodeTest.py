#!/usr/bin/env python

import rospy
from rtabmap_ros.srv import SetGoal
from rtabmap_ros.srv import SetGoalResponse

#Handler called when request received from service client 
def handleSplamNodeTest(req):
    #Printing arguments of the request
    print "Receiving node_id : %s"%(req.node_id) 
    print "Receiving node_label : %s"%(req.node_label)

    #Creating a test response to send back to service client
    resp = SetGoalResponse()   
    return resp

def splamNodeTest():
    #Node name
    rospy.init_node('splamNodeTest')

    #Declaring that this node will provide a service as a server
    s = rospy.Service('splamNodeTest', SetGoal, handleSplamNodeTest)
    print "SPLAM NODE TEST SERVER READY"
    
    #Starting node server 
    rospy.spin()

if __name__ == "__main__":
    splamNodeTest()





