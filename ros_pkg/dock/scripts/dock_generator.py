#!/usr/bin/env python

"""Provide docking motivation layer node."""

import rospy
import json
import math
from enum import Enum
from threading import Lock
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from hbba_msgs.msg import Desire
from hbba_msgs.msg import Intention
from hbba_msgs.srv import AddDesires
from hbba_msgs.srv import RemoveDesires


class DockState(Enum):
    """Define dockig process states."""

    idle = 0            # No dock required
    approaching = 1     # Driving to dock approach goal
    docking = 2         # Docking to station
    docked = 3          # Docked at station and dock required


class DockGenerator:
    """Generate dock desires according to robot and dock state."""

    def force_callback(self, msg):
        """Receive force docking command from electron."""
        self.lock.acquire()
        self.force_docking = msg.data
        self.last_force = rospy.Time.now()
        self.lock.release()

    def battery_callback(self, msg):
        """Receive battery level from battery board."""
        self.lock.acquire()
        if msg.percentage < self.min_percent:
            self.low_battery = True
        if msg.percentage > self.charge_percent:
            self.low_battery = False
        self.lock.release()

    def collision_callback(self, msg):
        """Receive collision events from collision detector."""
        self.lock.acquire()

        if self.state == DockState.docking:
            self.finish_docking()

        self.lock.release()

    def intention_callback(self, msg):
        """Monitor current robot intention from hbba."""
        self.lock.acquire()

        self.approach_active = self.approach_desire.id in msg.desires

        if self.state == DockState.docking:
            if self.loiter_desire.id in msg.desires:
                self.state = DockState.docked

            elif self.dock_desire.id not in msg.desires:
                self.restart_docking()

        if self.state == DockState.docked:
            if self.loiter_desire.id not in msg.desires:
                self.stop_docking()

        self.lock.release()

    def approach_callback(self, msg):
        """Set approach goal."""
        self.lock.acquire()

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

        self.lock.release()

    def start_docking(self):
        """Initialize docking sequence."""
        self.add_desire([self.dock_desire])
        self.restart_docking()

    def restart_docking(self):
        """Restart approach after interuption."""
        self.approach_desire.params = json.dumps(self.approach_goal)
        self.add_desire([self.approach_desire])

        self.state = DockState.approaching

    def finish_approach(self):
        """End approach phase."""
        self.rem_desire([self.approach_desire.id])
        self.state = DockState.docking

    def finish_docking(self):
        """End docking phase."""
        self.add_desire([self.loiter_desire])
        self.rem_desire([self.dock_desire.id])

    def stop_docking(self):
        """Cancel docking process."""
        if self.state == DockState.approaching:
            # both dock and approach must be removed
            self.rem_desire([
                self.dock_desire.id,
                self.approach_desire.id
            ])
        elif self.state == DockState.docking:
            # only dock desire must be removed
            self.rem_desire([self.dock_desire.id])
        elif self.state == DockState.docked:
            # loiter desire must be removed
            self.rem_desire([self.loiter_desire.id])

        self.state = DockState.idle

    def run(self):
        """Update desires and check if robot reached approach goal."""
        rospy.Subscriber('force_docking', Bool, self.force_callback)
        rospy.Subscriber('collision', Empty, self.collision_callback)
        rospy.Subscriber('battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('approach_goal', PoseStamped, self.approach_callback)
        rospy.Subscriber('hbba/intention', Intention, self.intention_callback)

        tf_listener = tf.TransformListener()
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()
            self.lock.acquire()

            # Check if force is expired
            now = rospy.Time.now()
            timeout = rospy.Duration(self.force_timeout)
            if now - self.last_force > timeout:
                self.force_docking = False

            # Check if we need to start docking
            if (self.force_docking or self.low_battery)\
               and self.state == DockState.idle:
                self.start_docking()
                self.lock.release()
                continue

            # Check if we need to stop docking
            if not self.force_docking\
               and not self.low_battery\
               and self.state != DockState.idle:
                self.stop_docking()
                self.lock.release()
                continue

            # We are not approaching we cancel tf lookup
            if self.state != DockState.approaching or not self.approach_active:
                self.lock.release()
                continue

            # Lookup robot pose in map
            try:
                pos, quat = tf_listener.lookupTransform(
                    self.map_frame,
                    self.robot_frame,
                    rospy.Time())
            except (tf.Exception):
                self.lock.release()
                continue

            # Check if we reached our approach goal
            dx = pos[0] - self.approach_goal['x']
            dy = pos[1] - self.approach_goal['y']
            lin_dst = math.sqrt(dx*dx + dy*dy)

            angles = euler_from_quaternion(quat)
            yaw_dst = angles[2] - self.approach_goal['t']

            if lin_dst < self.appr_lin_tol and abs(yaw_dst) < self.appr_yaw_tol:
                self.finish_approach()

            self.lock.release()

    def __init__(self):
        """Initialize dock generator node."""
        self.lock = Lock()
        self.low_battery = False
        self.force_docking = False
        self.approach_active = False
        self.state = DockState.idle
        self.last_force = rospy.Time()

        rospy.init_node('dock_generator')

        rospy.loginfo('waiting for add desire to be availble')
        rospy.wait_for_service('/hbba/add_desires')
        self.add_desire = rospy.ServiceProxy(
            '/hbba/add_desires',
            AddDesires)
        rospy.loginfo('add desire registered')

        rospy.loginfo('waiting for remove desire to be availbe')
        rospy.wait_for_service('/hbba/remove_desires')
        self.rem_desire = rospy.ServiceProxy(
            '/hbba/remove_desires',
            RemoveDesires)
        rospy.loginfo('remove desire registered')

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')

        self.appr_lin_tol = rospy.get_param('~appr_lin_tol', 0.40)
        self.appr_yaw_tol = rospy.get_param('~appr_yaw_tol', 0.05)

        self.min_percent = rospy.get_param('~min_percent', 30)
        self.charge_percent = rospy.get_param('~charge_percent', 80)
        self.undock_time = rospy.get_param('~undock_time', 4.0)
        self.force_timeout = rospy.get_param('~force_timeout', 10.0)

        self.approach_goal = dict()
        self.approach_goal['x'] = rospy.get_param('approach/x', 0.0)
        self.approach_goal['y'] = rospy.get_param('approach/y', 0.0)
        self.approach_goal['t'] = rospy.get_param('approach/t', 0.0)
        self.approach_goal['frame_id'] = self.map_frame

        self.approach_desire = Desire()
        self.approach_desire.id = 'dock_approach'
        self.approach_desire.type = 'GoTo'
        self.approach_desire.intensity = 1
        self.approach_desire.utility = 1
        self.approach_desire.security = False

        self.dock_desire = Desire()
        self.dock_desire.id = 'dock_dock'
        self.dock_desire.type = 'Dock'
        self.dock_desire.intensity = 1
        self.dock_desire.utility = 1
        self.dock_desire.security = False

        self.loiter_desire = Desire()
        self.loiter_desire.id = 'dock_charge'
        self.loiter_desire.type = 'Loiter'
        self.loiter_desire.intensity = 1
        self.loiter_desire.utility = 1
        self.loiter_desire.security = False

        params = dict()
        params['t'] = self.undock_time
        self.loiter_desire.params = json.dumps(params)


def node():
    """Run the docking motivation node."""
    generator_node = DockGenerator()
    generator_node.run()


if __name__ == '__main__':
    node()
