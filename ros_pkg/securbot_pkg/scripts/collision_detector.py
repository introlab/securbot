#! /usr/bin/python

import rospy
import tf
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import message_filters


class CollisionDetector:
    def sync_callback(self, cmd, odom):
        if abs(cmd.linear.x - odom.twist.twist.linear.x) > self.delta_max:
            if self.over_counter >= 5:
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

        tf_listener = tf.TransformListener()
        self.tag_vx = 0
        self.tag_counter = 0
        last_px = 0
        last_transform = rospy.Time(0)

        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                tf_listener.waitForTransform(
                    "tag_1",
                    "base_footprint",
                    now,
                    rospy.Duration(5.0)
                )
                pos, _ = tf_listener.lookupTransform(
                    "tag_1",
                    "base_footprint",
                    now)

                self.tag_vx = (pos[0] - last_px) / (now - last_transform).to_sec()
                last_px = pos[0]
                last_transform = now

                rospy.loginfo("tag vx %04.2f m/s" % self.tag_vx)

            except tf.Exception:
                continue


if __name__ == '__main__':
    CollisionDetector()
