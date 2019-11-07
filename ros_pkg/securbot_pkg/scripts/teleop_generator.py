#! /usr/bin/python

import rospy
import json
from std_msgs.msg import String
from hbba_msgs.msg import Desire
from hbba_msgs.srv import AddDesires
from hbba_msgs.srv import RemoveDesires


class TeleopGenerator:
    def __init__(self):
        rospy.init_node('teleop_generator')

        self.teleop_desire = Desire()
        self.teleop_desire.id = rospy.get_param('~des_id', 'teleop_desire')
        self.teleop_desire.type = 'Teleop'
        self.teleop_desire.utility = 1
        self.teleop_desire.security = False
        self.teleop_desire.intensity = rospy.get_param('~des_intensity', 1)

        self.add_desires = rospy.ServiceProxy(
            'add_desires',
            AddDesires
        )
        self.remove_desires = rospy.ServiceProxy(
            'remove_desires',
            RemoveDesires
        )

        rospy.Subscriber('fromElectron', String, self.electron_callback)
        rospy.spin()

    def electron_callback(self, msg):
        data = json.loads(msg.data)

        if data['enabled']:
            self.add_desires([self.teleop_desire])
        else:
            self.remove_desires([self.teleop_desire.id])


if __name__ == '__main__':
    TeleopGenerator()
