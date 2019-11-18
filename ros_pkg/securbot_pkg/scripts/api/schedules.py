"""Provide schedules API."""

import requests
from api import api_url
from rospy import Time
from rospy import Duration


class Schedule:
    def __init__(self):
        self.robot = ""
        self.name = ""
        self.description = ""
        self.last_modified = Time()
        self.patrol = ""
        self.cron = ""
        self.timeout = Duration()
        self.repetitions = 0
        self.enabled = False


def get(robot_id):
    request = requests.get(api_url + '/schedules')
    payload = request.json()
