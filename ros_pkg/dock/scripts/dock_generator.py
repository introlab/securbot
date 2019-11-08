#!/usr/bin/env python

import rospy
import json
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from hbba_msgs.msg import Desire
from hbba_msgs.srv import AddDesires
from hbba_msgs.srv import RemoveDesires


class DockGenerator:
    def force_callback(self, msg):
        # Reset collision if new force command
        if msg.data and not self.force:
            self.docked = False

        self.force = msg.data
        self.set_desire()

    def battery_callback(self, msg):
        if msg.percentage < self.min_percent:
            self.low_battery = True
            self.docked = False
        if msg.percentage > self.charge_percent:
            self.low_battery = False
        self.set_desire()

    def collision_callback(self, msg):
        self.docked = True
        self.set_desire()

    def approach_callback(self, msg):
        self.approach_goal['frame_id'] = msg.header.frame_id
        self.approach_goal['x'] = msg.pose.position.x
        self.approach_goal['y'] = msg.pose.position.y

        quat = (
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        )
        euler = euler_from_quaternion(quat)
        self.approach_goal['t'] = euler[2]

    def set_desire(self):
        if self.force or self.low_battery:
            if self.docked:
                self.add_desire([self.loiter_desire])
                self.rem_desire([self.dock_desire.id])
            else:
                self.dock_desire.params = json.dumps(self.approach_goal)
                self.add_desire([self.dock_desire])
                self.rem_desire([self.loiter_desire.id])
        else:
            self.rem_desire([self.dock_desire.id, self.loiter_desire.id])

    def __init__(self):
        self.in_progress = False
        self.docked = False
        self.force = False
        self.low_battery = False

        rospy.init_node('dock_generator')

        self.add_desire = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desire = rospy.ServiceProxy('remove_desires', RemoveDesires)

        rospy.Subscriber('force_docking', Bool, self.force_callback)
        rospy.Subscriber('collision', Empty, self.collision_callback)
        rospy.Subscriber('battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('approach_goal', PoseStamped, self.approach_callback)

        self.min_percent = rospy.get_param('~min_percent', 30)
        self.charge_percent = rospy.get_param('~charge_percent', 80)

        self.approach_goal = dict()
        self.approach_goal['x'] = rospy.get_param('~approach/x', 0.0)
        self.approach_goal['y'] = rospy.get_param('~approach/y', 0.0)
        self.approach_goal['z'] = rospy.get_param('~approach/t', 0.0)
        self.approach_goal['frame_id'] = rospy.get_param(
            '~approach/frame_id',
            '')

        self.dock_desire = Desire()
        self.dock_desire.id = 'dock_desire'
        self.dock_desire.type = 'Dock'
        self.dock_desire.intensity = 1
        self.dock_desire.utility = 1
        self.dock_desire.security = False

        self.loiter_desire = Desire()
        self.loiter_desire.id = 'dock_loiter'
        self.loiter_desire.type = 'Loiter'
        self.loiter_desire.intensity = 1
        self.loiter_desire.utility = 1
        self.loiter_desire.security = False

        rospy.spin()


def node():
    generator_node = DockGenerator()


if __name__ == '__main__':
    node()
