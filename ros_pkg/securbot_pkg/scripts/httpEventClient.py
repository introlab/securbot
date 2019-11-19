#!/usr/bin/python

import rospy
import os
import platform
import sys
import requests 
import json
import tf
from datetime import timedelta
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Image


class HttpEventClient:
    """
    HTTP Client that post events received from event detection to the database.

    """
    def __init__(self):
        self.api_url = ''
        self.robot_id = ''
        self.post_failed = 0
        self.event = ''
        self.event_time = ''
        self.robot = {
            "name": "",
            "platform": {
                "OS": "",
                "ROS": "",
                "CPU": "",
                "GPU": "",
                "Batteries": "",
                "Frame": "",
            }
        }
        rospy.init_node("http_event_client")

        rospy.Subscriber("/event/infos", String, self.detection_callback)
        rospy.Subscriber("/event/image", Image, self.frame_callback)

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')

        try:
            self.build_robot()
        except KeyError, e:
            rospy.logerr("Missing %s environement variable" % e.args[0])
            exit(-1)

        try:
            robots = self.get_robots_id()
            for robot in robots:
                if robot["name"] == self.robot["name"]:
                    self.robot_id = robot["_id"]
                    break

            if self.robot_id == "":
                self.post_self()
            else:
                self.update_self()
        except requests.exceptions.RequestException, e:
            rospy.logerr("Failed to retrieve robot because %s", e.args[0])
            exit(-1)

        rospy.spin()

    def build_robot(self):
        self.robot["name"] = os.environ["SECURBOT_ROBOT_NAME"]
        _os = platform.linux_distribution()
        self.robot["platform"]["OS"] = _os[0] + " " + _os[1]
        self.robot["platform"]["ROS"] = rospy.get_param("rosdistro")[0:-1] + " " + rospy.get_param("rosversion")[0:-1]
        self.robot["platform"]["CPU"] = platform.processor()
        self.robot["platform"]["GPU"] = "Nvidia"
        self.robot["platform"]["Batteries"] = ""
        self.robot["platform"]["Frame"] = os.environ["SECURBOT_BASE"]


        server_url = os.environ["SECURBOT_SERVER_URL"]
        api_path = os.environ["SECURBOT_API_PATH"]
        self.api_url = server_url + api_path

    def get_robots_id(self):
        request = requests.get(self.api_url + "/robots", timeout=10)
        request.raise_for_status()
        return request.json()

    def post_self(self):
        request = requests.post(self.api_url + "/robots", self.robot)
        request.raise_for_status()
        if request.status_code == 200:
            self.robot_id = request.json()["_id"]
        else:
            rospy.logerr("Failed to add itself onto the database")
            exit(-1)

    def update_self(self):
        request = requests.put(self.api_url + "/robots/" + self.robot_id, self.robot)
        request.raise_for_status()
        if request.status_code != 200:
            rospy.logerr("Failed to update itself onto the database")
            exit(-1)

    def post_event(self):
        request = requests.post(self.api_url + "/robots/" + self.robot_id + "/events", self.event)
        request.raise_for_status()
        if request.status_code != 200:
            rospy.logerr("Failed to post the event onto the database")
            if self.post_failed < 3:
                rospy.logerr("Trying to post the event again")
                self.post_failed = self.post_failed + 1
                self.post_event()
            else:
                rospy.logerr("Failed to post the event onto the database more that 3 times. Discarding the event...")
                self.event = ''
        else:
            self.event = ''

    # def post_img_of_event(self, event):
    #     request = requests.post(self.api_url + "/robots/" + self.robot_id, event)
    #     request.raise_for_status()

    def detection_callback(self, e):
        if self.event_time:
            time = timedelta().total_seconds() - self.event_time.total_seconds()
            if time > 5:
                rospy.logerr("Failed to received an image for the event in less than 5sec, posting the event without image.")
                self.event_time = ''
                self.post_event()

        if self.event:
            rospy.logerr("An new event was detected while the previous one still hasn't received its image. Ignoring new event.")
        else:
            info = json.loads(e)
            coord = self.get_current_position()
            tags = self.get_current_tags()
            self.event = {
                "robot": self.robot_id,
                "object": info[0] + " Event",
                "description_text": "",
                "context": info[2],
                "files": "",
                "time": info[3],
                "coordinate": coord,
                "tags": tags,
                "alert": False,
            }
            rospy.loginfo(json.dumps(self.event))
            self.event_time = timedelta()
            

    def frame_callback(self, image):
        if self.event:
            rospy.loginfo("Received an image for event " + self.event["object"])
            self.event["files"] = image
            self.post_event()
            
        
        # rospy.loginfo(json.dumps(image))

    def get_current_position(self):
        tf_listener = tf.TransformListener()
        try:
            pos, quat = tf_listener.lookupTransform(
                self.map_frame,
                self.robot_frame,
                rospy.Time())
            return {"x": pos[0], "y": pos[1], "yaw": euler_from_quaternion(quat)[2]}
        except (tf.Exception):
            rospy.logerr("Failed to lookup current transform, the values will be set to -1")
            return {"x": -1, "y": -1, "yaw": 0}

    def get_current_tags(self):
        pass


if __name__ == '__main__':
    HttpEventClient()
