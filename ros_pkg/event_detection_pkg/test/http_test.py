#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
blank_image = np.zeros((100,100,3), np.uint8)

pub = rospy.Publisher ('/event_detection/event_detection', String, queue_size=10)
image_pub = rospy.Publisher ('/event_detection/detection_frame', Image, queue_size=10)
rospy.init_node('http_test')
rospy.loginfo('starting test')
pub.publish('["nom", "number", "2019-11-19T17:23:41+0000"]')
image_pub.publish(bridge.cv2_to_imgmsg(blank_image))
