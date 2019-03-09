#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

def electronNodeTest():
    pub = rospy.Publisher('fromElectron', String, queue_size=10)
    rospy.init_node('electronNodeTest', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        waypointTest = "   %s   " % rospy.get_time()
        rospy.loginfo(waypointTest)
        pub.publish(waypointTest)
        rate.sleep()

if __name__ == '__main__':
    try:
        electronNodeTest()
    except rospy.ROSInterruptException:
        pass
