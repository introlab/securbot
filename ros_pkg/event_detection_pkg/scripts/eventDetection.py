#!/usr/bin/python

import rospy, json, hashlib, actionlib, math
import datetime
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

defaultEventConfigDictList = [{
                              "event_name": "face_recognition",
                              "active": True,
                              "startTime": None,
                              "stopTime": None,
                              "threshold": "face"
                            }]

class EventDetection:
    def __init__(self, defaultEventConfigDictList = None):
        rospy.loginfo(datetime.datetime.now().strftime("[%H:%M:%S]")\
                + "[DEBUG]: Initializing Event Detection Node...")

        # Visual event detection flag for forwading a capture of the video feed
        self.hasDetectedVisualEvent = False
        self.eventTimeout = 10
        self.prevEventTime = time.time()

        # Events Config List
        self.eventsConfigDictList = list()
        if(defaultEventConfigDictList != None and isinstance(defaultEventConfigDictList,list)):
            self.eventsConfigDictList = defaultEventConfigDictList

        # Node name defined as eventDetection
        rospy.init_node("eventDetection", anonymous=True)
        # anonymous=True keeps each eventDetection nodes unique if
        # there were many

        # Subscribing to topic 'bounding_boxes' with callback
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.boundingBoxesCallback)

        # Subscribing to topic 'detection_image' with callback
        rospy.Subscriber("darknet_ros/detection_image", Image, self.detectionImageCallback)

        #TODO: For future Securbot version, support configuration request on the end-device/web client side
        # Subscribing to topic 'event_detection_config' with callback
        #rospy.Subscriber("/event_detection/event_detection_config", String, self.eventDetectionConfigCallback)

        # Publishing to topic 'event_detection'
        self.t_eventDetection = rospy.Publisher("event_detection/event_detection", String, queue_size = 20)
        # Publishing to topic 'detection_frame'
        self.t_detectionFrame = rospy.Publisher("event_detection/detection_frame", Image, queue_size = 20)

        rospy.spin()

    def stampEventNameDateTime(self, eConfig, probability):
        #Date and time's format as ISO 8601 UTC
        eventStamp = {
                        "event_name": eConfig["event_name"],
                        "probability" : str(probability),
                        "datetime" : datetime.datetime.utcnow().replace(microsecond=0).isoformat()+'Z'
                     }
        return eventStamp

    def boundingBoxesCallback(self, boundingBoxes):
        if time.time() - self.prevEventTime > self.eventTimeout:
            rospy.loginfo(datetime.datetime.now().strftime("[%H:%M:%S]")\
            + " [DEBUG]: Visual Object Detected!")

            #Parse every detected classes
            bboxDictList = list()
            for bbox in boundingBoxes.bounding_boxes:
                bboxDict = dict()
                bboxDict["class"] = bbox.Class
                bboxDict["probability"] = bbox.probability
                bboxDictList.append(bboxDict)

            #Check if any active events
            for eConfig in self.eventsConfigDictList:
                if(eConfig["active"] == True):
                    if( (eConfig["startTime"] == None                             \
                                            and                                   \
                        eConfig["stopTime"] == None)                             \
                                            or                                    \
                        (eConfig["startTime"] <= datetime.datetime.now().time()  \
                                            and                                   \
                        eConfig["stopTime"] >= datetime.datetime.now().time())                  ):
                        for bboxDict in bboxDictList:
                            #Check if threshold reached
                            if(eConfig["threshold"] == bboxDict["class"]):
                                triggeredEvent = self.stampEventNameDateTime(eConfig, bboxDict["probability"])
                                self.t_eventDetection.publish(json.dumps(triggeredEvent))
                                self.hasDetectedVisualEvent = True
                                self.prevEventTime = time.time()
                                break



    def detectionImageCallback(self, img):
        if(self.hasDetectedVisualEvent == True):
            rospy.loginfo(datetime.datetime.now().strftime("[%H:%M:%S]")\
                + " [DEBUG]: Publishing Frame!")
            self.t_detectionFrame.publish(img)
            self.hasDetectedVisualEvent = False

    def addEventConfig(self, eConfig):
        hasSameEventName = False
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                hasSameEventName = True
        if(hasSameEventName == False):
            self.eventsConfigDictList.append(eConfig)

    def modifyEventConfig(self, eConfig):
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                #Check if the name needs to change
                if(eConfig.get("modify_event_name") == None):
                    ecd = eConfig
                else:
                    modifiedName = eConfig.get("modify_event_name")
                    eConfig.pop("modify_event_name")#Copy without new name key/value
                    ecd = eConfig
                    ecd["event_name"] = modifiedName

    def deleteEventConfig(self, eConfig):
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                self.eventsConfigDictList.remove(ecd)

    def eventDetectionConfigCallback(self, eventConfig):
        eConfig = json.loads(eventConfig)
        if(eConfig.get("config_type") != None):
            if(eConfig.get("event_name") != None):
                if(eConfig["config_type"] == "add"):
                    eConfig.pop("config_type")
                    self.addEventConfig(eConfig)
                elif(eConfig["config_type"] == "modify"):
                    eConfig.pop("config_type")
                    self.modifyEventConfig(eConfig)
                elif(eConfig["config_type"] == "delete"):
                    eConfig.pop("config_type")
                    self.deleteEventConfig(eConfig)
                elif(eConfig["config_type"] == "clear_all"):
                    self.eventsConfigDictList.clear()
            else:
                rospy.loginfo(datetime.datetime.now().strftime("[%H:%M:%S]")\
                        + "[WARNING] : Missing 'event_name' key while configuring...")
        else:
            rospy.loginfo(datetime.datetime.now().strftime("[%H:%M:%S]")\
                    + "[WARNING] : Missing 'config_type' key while configuring...")

if __name__ == "__main__":
    ed = EventDetection(defaultEventConfigDictList)
