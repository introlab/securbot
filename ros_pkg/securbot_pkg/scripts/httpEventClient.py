#!/usr/bin/python

import rospy
import os
import platform
import requests
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image


class HttpEventClient:
    """
    HTTP Client that post events received from event detection to the database.

    """
    class Robot:
        id = ''
        name = ''
        class platform:
            OS = platform.linux_distribution
            ROS = ''
            CPU = platform.processor
            GPU = ''
            Batteries = ''
            Frame = ''

    def __init__(self):
        rospy.init_node("http_event_client")

        rospy.Subscriber('/event_detection/event_detection', String, self.detection_callback)
        rospy.Subscriber('/event_detection/detection_frame', Image, self.frame_callback)

        # Get robot name and server url from environment variables
        try:
            self.Robot.name = os.environ["SECURBOT_ROBOT_NAME"]

            server_url = os.environ["SECURBOT_SERVER_URL"]
            api_path = os.environ["SECURBOT_API_PATH"]
            self.api_url = server_url + api_path

        except KeyError, e:
            rospy.logerr("Missing %s environement variable" % e.args[0])
            exit(-1)

        try:
            robots = self.get_robots_id()
            for robot in robots:
                if robot["name"] == self.Robot.name:
                    self.Robot.id = robot["_id"]
                    break

                if self.Robot.id == "":
                    self.post_myself()
        except requests.exceptions.RequestException, e:
            rospy.logerr("Failed to retrieve robot because %s", e.args[0])
            return

    def get_robots_id(self):
        request = requests.get("%s/robots" % self.api_url, timeout=10)
        request.raise_for_status()
        return request.json()

    def post_myself(self):
        request = requests.post("%s/robots" % self.api_url, json.dumps(self.Robot))
        request.raise_for_status()
        robot = request.json()
        self.Robot.id = robot["_id"]

    def post_event(self, event):
        request = requests.post(self.api_url + "/robots/%s" + self.Robot.id, json.dumps(event))
        request.raise_for_status()
        return request.json()

    def update_event(self, event):
        request = requests.post(self.api_url + "/robots/%s" + self.Robot.id, json.dumps(event))
        request.raise_for_status()
        return request.json()

    def detection_callback(self, event):
        rospy.loginfo(json.dumps(event))

    def frame_callback(self, image):
        rospy.loginfo('Received an image...')
        # rospy.loginfo(json.dumps(image))


if __name__ == "__main__":
    node = HttpEventClient()
