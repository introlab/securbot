#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

#include <algorithm>

int target_tag = -1;

void DetectionArrayCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{

    for (int i; i < msg->detections.size(); ++i)
    {
        const apriltag_ros::AprilTagDetection & detection = msg->detections[i];

        if (std::count(detection.id.begin(), detection.id.end(), target_tag))
        {
            ROS_INFO("Found suitable candidate");
            return;
        }
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "dock");

    ros::NodeHandle nh;

    // Parameters
    nh.param("dock/tag_id", target_tag, 0);

    // Subscribed topics
    ros::Subscriber sub = nh.subscribe("detect_array", 100, DetectionArrayCallback);

    ros::spin();

    return 0;
}
