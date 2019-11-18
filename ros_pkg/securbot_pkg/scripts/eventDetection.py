#!/usr/bin/python

import rospy, json, hashlib, actionlib, math
from datetime import date, datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

class EventDetection:
    def __init__(self):
        # Visual event detection flag for forwading a capture of the video feed
        self.hasDetectedVisualEvent = False

        # Events Config List
        self.eventsConfigDictList = list()

        # Node name defined as eventDetection
        rospy.init_node("eventDetection", anonymous=True)
        # anonymous=True keeps each eventDetection nodes unique if
        # there were many

        # Subscribing to topic 'bounding_boxes' with callback
        rospy.Subscriber("/darknet_ros_msgs/bounding_boxes", BoundingBoxes,
                self.boundingBoxesCallback)

        # Subscribing to topic 'detection_image' with callback
        rospy.Subscriber("/darknet_ros_msgs/detection_image", Image,
                self.detectionImageCallback)

        # Subscribing to topic 'event_detection_config' with callback
        rospy.Subscriber("/event_detection/event_detection_config", String,
                self.eventDetectionConfigCallback)

        # Publishing to topic 'event_detection'
        self.t_eventDetection = rospy.Publisher("/event_detection/event_detection",
            String, queue_size = 20)
        # Publishing to topic 'detection_frame'
        self.t_detectionFrame = rospy.Publisher("/event_detection/detection_frame",
                Image, queue_size =20)

        rospy.spin()

    def stampEventNameDateTime(eConfig, probability):
        #Date and time's format as ISO 8601
        eventStamp = {
                        "event_name": eConfig["event_name"],
                        "probability" : probability,
                        "date" : date.today().strftime("%Y/%m/%d"),
                        "time" : datetime.now().time().strftime("%H:%M:%S")
                     }
        return eventStamp

    def boundingBoxesCallback(boundingBoxes):
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
                if( eConfig["startTime"] <= datetime.now().time()   \
                                        and                         \
                                        eConfig["stopTime"] >= datetime.now().time()):
                    for bboxDict in bboxDictList:
                        #Check if threshold reached
                        if(eConfig["threshold"] == bboxDict["class"]):
                            triggeredEvent = self.stampEventNameDateTime(eConfig, bboxDict["probability"])
                            self.t_eventDetection.pulish(json.dumps(triggeredEvent))
                            self.hasDetectedVisualEvent = True

    def detectionImageCallback(img):
        if(self.hasDetectedVisualEvent == True):
            self.t_detectionFrame.publish(img)
            self.hasDetectedVisualEvent = False

    def addEventConfig(eConfig):
        hasSameEventName = False
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                hasSameEventName = True
        if(hasSameEventName == False):
            self.eventsConfigDictList.append(eConfig)

    def modifyEventConfig(eConfig):
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                #Check if the name needs to change
                if(eConfig.get("modify_event_name") == None):
                    ecd = eConfig
                else:
                    modifiedName = eConfig.get("modify_event_name")
                    ecd = eConfig.pop("modify_event_name")#Copy without new name
                    ecd["event_name"] = modifiedName

    def deleteEventConfig(eConfig):
        for ecd in self.eventsConfigDictList:
            if(ecd["event_name"] == eConfig["event_name"]):
                self.eventsConfigDictList.remove(ecd)

    def eventDetectionConfigCallback(eventConfig):
        eConfig = json.loads(eventConfig)
        if(eConfig.get("config_type") != None):
            if(eConfig.get("event_name") != None):
                if(eConfig["config_type"] == "add"):
                    self.addEventConfig(eConfig)
                elif(eConfig["config_type"] == "modify"):
                    self.modifyEventConfig(eConfig)
                elif(eConfig["config_type"] == "delete"):
                    self.deleteEventConfig(eConfig)
                elif(eConfig["config_type"] == "clear_all"):
                    self.eventsConfigDictList.clear()
            else:
                rospy.loginfo(datetime.now().strftime("[%H:%M:%S]")\
                        + "[ERROR] : Missing 'event_name' key while configuring...")
        else:
            rospy.loginfo(datetime.now().strftime("[%H:%M:%S]")\
                    + "[ERROR] : Missing 'config_type' key while configuring...")

if __name__ == "__main__":
    ed = EventDetection()
