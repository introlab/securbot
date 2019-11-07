#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from hbba_msgs.msg import Desire
from hbba_msgs.srv import AddDesires
from hbba_msgs.srv import RemoveDesires


class DockGenerator:
    def force_callback(self, msg):
        self.force = msg.data
        self.set_desire()

    def battery_callback(self, msg):
        self.low_battery = msg.percentage < self.min_percent
        self.set_desire()

    def set_desire(self):
        if self.force or self.low_battery:
            self.add_desire([self.dock_desire])
        else:
            self.rem_desire([self.des_id])

    def __init__(self):
        self.force = False
        self.low_battery = False

        rospy.init_node('dock_generator')

        rospy.Subscriber('force_docking', Bool, self.force_callback)
        rospy.Subscriber('battery_state', BatteryState, self.battery_callback)

        self.add_desire = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desire = rospy.ServiceProxy('remove_desires', RemoveDesires)

        self.min_percent = rospy.get_param('~min_percent', 30)
        self.des_id = rospy.get_param('~des_id', 'dock_desire')
        self.des_int = rospy.get_param('~des_intensity', 1)

        self.dock_desire = Desire()
        self.dock_desire.id = self.des_id
        self.dock_desire.type = 'Dock'
        self.dock_desire.intensity = self.des_int
        self.dock_desire.utility = 1
        self.dock_desire.security = False

        rospy.spin()


def node():
    generator_node = DockGenerator()


if __name__ == '__main__':
    node()
