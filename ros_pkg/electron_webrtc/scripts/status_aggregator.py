#! /usr/bin/python

import rospy
import json
import psutil
from collections import namedtuple
from threading import Lock
from std_msgs.msg import String
from env_sensors.msg import WifiSignal
from hbba_msgs.msg import Intention


class StatusAggregator:
    def __init__(self):
        self.wifi_lock = Lock()
        self.wifi = dict()

        self.intention_lock = Lock()
        self.intention = dict()

        rospy.init_node("status_aggregator")

        self.status_publisher = rospy.Publisher(
            "robot_status",
            String,
            queue_size=1)

        rospy.Subscriber("wifi_level", WifiSignal, self.wifi_callback)
        rospy.Subscriber("hbba/intention", Intention, self.intention_callback)

        self.publishLoop()

    def publishLoop(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            rate.sleep()

            res = dict()
            res["cpu"] = psutil.cpu_percent()
            res["mem"] = psutil.virtual_memory().percent

            self.wifi_lock.acquire()
            wifi = self.wifi
            self.wifi_lock.release()

            self.intention_lock.acquire()
            intention = self.intention
            self.intention_lock.release()

            status = dict()
            status["resources"] = res
            status["wifi"] = wifi
            status["intention"] = intention

            status_str = json.dumps(status)
            self.status_publisher.publish(status_str)

    def wifi_callback(self, msg):
        wifi = dict()
        wifi["rssi"] = msg.rssi
        wifi["noise"] = msg.noise

        self.wifi_lock.acquire()
        self.wifi = wifi
        self.wifi_lock.release()

    def intention_callback(self, msg):
        strategies = list()
        desires = list()

        for i, strat_id in enumerate(msg.strategies):
            if msg.enabled[i]:
                strategies.append(strat_id)

            if msg.desires[i] != "":
                des = dict()
                des["id"] = msg.desires[i]
                des["type"] = msg.desire_types[i]
                desires.append(des)

        intention = dict()
        intention['strategies'] = strategies
        intention['desires'] = desires

        self.intention_lock.acquire()
        self.intention = intention
        self.intention_lock.release()


if __name__ == "__main__":
    StatusAggregator()
