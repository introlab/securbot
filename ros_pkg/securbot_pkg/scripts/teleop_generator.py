#! /usr/bin/python

import rospy
import json
from threading import Lock
from std_msgs.msg import String
from hbba_msgs.msg import Desire
from hbba_msgs.srv import AddDesires
from hbba_msgs.srv import RemoveDesires


class TeleopGenerator:
    def __init__(self):
        rospy.init_node('teleop_generator')
        self.desire_active = False

        self.teleop_desire = Desire()
        self.teleop_desire.id = rospy.get_param('~des_id', 'teleop_desire')
        self.teleop_desire.type = 'Teleop'
        self.teleop_desire.utility = 1
        self.teleop_desire.security = False
        self.teleop_desire.intensity = rospy.get_param('~des_intensity', 2)

        self.add_desires = rospy.ServiceProxy(
            'add_desires',
            AddDesires
        )
        self.remove_desires = rospy.ServiceProxy(
            'remove_desires',
            RemoveDesires
        )

        self.last = rospy.Time.now()
        self.last_mutex = Lock()

        rospy.Subscriber('fromElectron', String, self.electron_callback)
        rospy.Timer(rospy.Duration(2), self.timer_callback)

        rospy.spin()

    def timer_callback(self, event):
        self.last_mutex.acquire()
        last = self.last
        active = self.desire_active
        self.last_mutex.release()

        delay = rospy.Time.now() - last

        if delay.to_sec() > 10 and active:
            self.remove_desires([self.teleop_desire.id])
            active = False

        self.last_mutex.acquire()
        self.desire_active = active
        self.last_mutex.release()

    def electron_callback(self, msg):
        self.last_mutex.acquire()
        self.last = rospy.Time.now()
        active = self.desire_active
        self.last_mutex.release()

        data = json.loads(msg.data)

        if data['enabled'] and not active:
            self.add_desires([self.teleop_desire])
            active = True
        elif not data['enabled'] and active:
            self.remove_desires([self.teleop_desire.id])
            active = False

        self.last_mutex.acquire()
        self.desire_active = active
        self.last_mutex.release()


if __name__ == '__main__':
    TeleopGenerator()
