#! /usr/bin/python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class Reverser:
    def time_callback(self, msg):
        time = msg.data
        duration = rospy.Duration(time)

        cmd_vel = Twist()
        cmd_vel.linear.x = -self.reverse_speed

        rate = rospy.Rate(10)
        start_at = rospy.Time.now()

        while rospy.Time.now() - start_at < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

        cmd_vel.linear.x = 0

        for i in range(5):
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

    def __init__(self):
        rospy.init_node('reverser')

        self.reverse_speed = rospy.get_param('~speed', 0.15)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('time', Float32, self.time_callback)

        rospy.spin()

if __name__ == '__main__':
    Reverser()
