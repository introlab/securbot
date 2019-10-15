#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <algorithm>

int target_tag = -1;


bool findTagPose(
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg,
        int tag,
        geometry_msgs::PoseWithCovarianceStamped * out_pose)
{
    for (int i = 0; i < msg->detections.size(); ++i)
    {
        const apriltag_ros::AprilTagDetection & detection = msg->detections[i];

        if (std::count(detection.id.begin(), detection.id.end(), target_tag))
        {
            *out_pose = detection.pose;
            return true;
        }
    }
    return false;
}

void DetectionArrayCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped targetPose;

    if (! findTagPose(msg, target_tag, &targetPose))
        return;

    // Compute target position based of tag

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "dock");

    ros::NodeHandle nh;

    // Parameters
    nh.param("dock/tag_id", target_tag, 0);
    ROS_INFO("Searching for tag #%d", target_tag);

    // Subscribed topics
    ros::Subscriber sub = nh.subscribe("detect_array", 0, DetectionArrayCallback);

    ros::spin();

    return 0;
}
