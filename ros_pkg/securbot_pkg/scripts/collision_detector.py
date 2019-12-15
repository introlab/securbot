#! /usr/bin/python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import message_filters


class CollisionDetector:
    def sync_callback(self, cmd, odom):
        if abs(cmd.linear.x - odom.twist.twist.linear.x) > self.delta_max:
            if self.over_counter >= 10:
                self.coll_pub.publish(Empty())
            else:
                self.over_counter = self.over_counter + 1
        else:
            self.over_counter = 0

    def __init__(self):
        rospy.init_node('collision_detector')
        self.delta_max = rospy.get_param('~delta_max', 0.08)
        self.over_counter = 0

        self.coll_pub = rospy.Publisher('collision', Empty, queue_size=10)

        cmd_sub = message_filters.Subscriber('cmd_vel', Twist)
        odom_sub = message_filters.Subscriber('odom', Odometry)

        sync_sub = message_filters.ApproximateTimeSynchronizer(
            [cmd_sub, odom_sub],
            20,
            0.01,
            True)
        sync_sub.registerCallback(self.sync_callback)

        rospy.spin()


if __name__ == '__main__':
    CollisionDetector()
