#! /usr/bin/python

"""Provides PatrolScheduler class."""

import rospy
import os
import requests
import pycron
import json
from std_msgs.msg import String


class PatrolScheduler:
    """Implement the Patrol Scheduler node.

    The node reads schedule from database and publish
    the scheduled patrols at the correct time.
    """

    def __init__(self):
        """Register node and topics to ROS."""
        rospy.init_node("patrol_scheduler")

        self.patrol_publisher = rospy.Publisher(
            "scheduled_patrol",
            String,
            queue_size=10)

        self.robot_id = ""

        # Get robot name and server url from environment variables
        try:
            self.robot_name = os.environ["SECURBOT_ROBOT_NAME"]

            server_url = os.environ["SECURBOT_SERVER_URL"]
            api_path = os.environ["SECURBOT_API_PATH"]
            self.api_url = server_url + api_path
        except KeyError, e:
            rospy.logerr("Missing %s environement variable" % e.args[0])
            exit(-1)

    def retrieve_robot_id(self):
        """Get robot id matching this robot name from server."""
        # Get robot id
        request = requests.get("%s/robots" % self.api_url, timeout=10)
        request.raise_for_status()
        response = request.json()

        for robot in response:
            if robot["name"] == self.robot_name:
                self.robot_id = robot["_id"]
                break

        if self.robot_id == "":
            return []

        rospy.loginfo("Robot id is %s" % self.robot_id)
        return self.robot_id

    def retrieve_schedules(self):
        """Get the schedules from the server."""
        self.retrieve_robot_id()

        request = requests.get(
            "%s/robots/%s/schedules" % (self.api_url, self.robot_id),
            timeout=10)
        request.raise_for_status()
        return request.json()

    def retrieve_patrol_by_id(self, id):
        """Get a patrol from the server by its id."""
        request = requests.get(
            "%s/robots/%s/patrols/%s" % (self.api_url, self.robot_id, id),
            timeout=10)
        request.raise_for_status()
        response = request.json()

        patrol = dict()
        patrol["patrol"] = response["waypoints"]
        patrol["id"] = id

        return patrol

    def process_loop(self):
        """Send the patrol that should start this minute if there is one."""
        schedules = []

        rospy.loginfo("Processing scheduled patrols")

        # Retrieve schedules from server
        try:
            schedules = self.retrieve_schedules()
        except requests.exceptions.RequestException, e:
            rospy.logerr("Failed to retrieve schedules because %s", e.args[0])
            return

        # Print schedules for debug
        for schedule in schedules:
            rospy.loginfo(
                "Patrol %s scheduled at %s" %
                (schedule["patrol"], schedule["cron"])
            )

        # Get patrol ids that should run this minute
        patrol_ids = [
            (schedule["patrol"], schedule["repetitions"])
            for schedule in schedules
            if pycron.is_now(schedule["cron"]) and schedule["enabled"]
        ]

        # Multiple patrols. This is unfair standards for robot so we warn
        if len(patrol_ids) > 1:
            rospy.logwarn("Multiples patrols scheduled at the same time!")

        # Retrieve patrol matching each id
        patrols = []
        for id, rep in patrol_ids:
            rospy.loginfo("Patrol %s must run now. Retrieving..." % id)
            try:
                patrol = self.retrieve_patrol_by_id(id)
                patrol["loop"] = rep
                patrols.append(patrol)
            except requests.exceptions.RequestException, e:
                rospy.logwarn(
                    "Failed to retrieve patrol %s because %s",
                    e.args[0])
            rospy.loginfo("Patrol %s retrieved" % id)

        # Publish patrol to ROS
        for patrol in patrols:
            self.patrol_publisher.publish(json.dumps(patrol))

    def run(self):
        """Start node."""
        process_rate = rospy.Rate(1.0/60.0)

        while not rospy.is_shutdown():
            process_rate.sleep()
            self.process_loop()


if __name__ == "__main__":
    node = PatrolScheduler()
    node.run()
