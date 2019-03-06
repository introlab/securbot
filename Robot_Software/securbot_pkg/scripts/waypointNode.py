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
from rtabmap_ros/SetGoal.srv import *

#Using service to talk to SPLAM
def setGoalClient(waypoint):
    rospy.loginfo("Sending waypoint from Electron to SPLAM")
    rospy.loginfo("Attempting to set goal :" + waypoint)
    rospy.wait_for_service('set_goal')
    try:
        set_goal_handler = rospy.ServiceProxy('set_goal',SetGoal)
        resp = set_goal(waypoint) # request : Set either node_id(int32)
                                  #           or node_label(string)
        return resp
    except rospy.ServiceException, e:
        print("Service call set_goal failed : %s"%e)

#Subscribing and listening to Electron Node
def setGoalClientCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "heard %s", data.data)
    setGoalClient(data.data)

def waypointListener():
    rospy.init_node('waypointListener', anonymous=True)
    rospy.Subscriber("waypoints", String, setGoalClientCallback)
    rospy.spin()

 __name__ == '__main__':
    waypointListener()