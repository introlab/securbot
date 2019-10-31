#!/usr/bin/env python
PKG = 'securbot_pkg'

import sys, json, unittest, rospy, time, actionlib
from std_msgs.msg import String
from sensor_msgs.msg import Image
import darknet_ros
#from securbot_pkg import EventDetection

class DatabaseRosMock:
    def __init__():
        pass
    def dataBaseEventReceiverCallback():
        pass
    def dataBaseFrameReceiverCallback():
        pass
class ElectronRosMock:
    def __init__():
        pass
    def publishEventDetectionConfig():
        pass

class  EventDetectionTestSuite(unittest.TestCase):
    def __init__(self, *args):
        super(EventDetectionTestSuite, self).__init__(*args)
        rospy.init_node('event_detection_test')

        # Event Detection Node Subscribed And Published Topics
        rospy.Subscriber('/darknet_ros_msgs/bounding_boxes', BoundingBoxes, self.boundingBoxesCallback)
        #rospy.Subscriber('/darknet_ros_msgs/object_detector', int8, self.objectDetectorCallback) # Not yet used
        rospy.Subscriber('/darknet_ros_msgs/detection_image', image, self.detectionImageCallback)

        rospy.Subscriber('/event_detection/event_detection_config', String, self.eventDetectionConfigCallback)

        self.t_eventDetection = rospy.Publisher('/event_detection/event_detection',
                String, queue_size = 20)
        self.t_detectionFrame = rospy.Publisher('/event_detection/detection_frame',
                image, queue_size =20)

        # Database Ros Mock
        dbrMock = DatabaseRosMock()
        rospy.Subscriber('/event_detection/event_detection', String, dataBaseEventReceiverCallback)
        rospy.Subscriber('/event_detection/detection_frame', image, dataBaseFrameReceiverCallback)

        # Electron Ros Mock
        erMock = ElectronRosMock()
        erMock.t_eventDetectionConfig = rospy.Publisher('/event_detection/event_detection_config',
                String, queue_size = 20)

    # Setting up the test environment before every test
    def setUp():
        pass

    #
    # TEST FUNCTIONS
    #
    def test_event_detection_sent():
        pass
    def test_detection_frame_sent():
        pass
    def test_event_detection_configuration():
        pass
    def test_event_detection_changing_configuration_meawhile():
        pass
    def test_event_configuration_two_classes
        pass

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'event_detection', EventDetectionTestSuite)
