#TODO: Try to understand how to not have this line below without having an error
#coding=utf-8

#!/usr/bin/python

"""
Copyright (c) 2019, Dominic Létourneau - IntRoLab - Universite de Sherbrooke,
SecurBot - Universite de Sherbrooke, All rights reserved. Redistribution and
use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
* Neither the name of the Universite de Sherbrooke nor the names of its
  contributors may be used to endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
from std_msgs.msg import String
from rtabmap_ros.srv import SetGoal

#TODO: Might use json module to send response through Electron node toward web client (topic anticipated : 'toElectron')
#import json

#For test usage
import random

#Using service SetGoal to talk to SPLAM
def waypointTestClient(waypointString):
    rospy.loginfo("Sending waypoint from Electron to rtab-map/SPLAM...")
    rospy.loginfo("Waypoint to send : " + waypointString)
    #Test service to use
    rospy.wait_for_service('splamNodeTest')
    try:
	#Create handler function acting as function set_goal(node_id, node_label)
        handleSplamNodeTest = rospy.ServiceProxy('splamNodeTest', SetGoal)
        #Send random data as node_id and actual waypoint string received as node_label just for test purposes
	resp = handleSplamNodeTest( random.randrange(1, 2**31-1) , waypointString.strip()) 
                                            						   
	#TODO : Figure out how to use SetGoal service response data toward web client? (SetGoalResponse() instance manipulation)
	return resp

    except rospy.ServiceException, e:
        print("Service call splamNodeTest failed : %s"%e)

#Subscribing and listening to Electron Node
def waypointTestClientCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + " heard    %s   ", msg.data)
    #Calling the service as a client to SetGoal server
    waypointTestClient(msg.data)

def waypointListener():
    #Node name defined as waypointNodeTest
    rospy.init_node('waypointNodeTest', anonymous=True)
    #Subscribing to topic 'fromElectron' with callback
    rospy.Subscriber("fromElectron", String, waypointTestClientCallback)
    rospy.spin()

if __name__ == '__main__':
    waypointListener()
