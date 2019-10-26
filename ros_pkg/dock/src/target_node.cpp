#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <string>


int g_targetTag = -1;
std::string g_target_frame_id;

tf2_ros::Buffer g_tfBuffer;



bool ComputeTarget(geometry_msgs::TransformStamped * targetTransform)
{

    // Applying transform from tags to contacts
    geometry_msgs::TransformStamped stationTransform;
    try {
        stationTransform = g_tfBuffer.lookupTransform(
                "map",
                "station_" + std::to_string(g_targetTag),
                ros::Time(0));
    }
    catch (...) { return false; }


    stationTransform.child_frame_id = g_target_frame_id;
    stationTransform.transform.translation.z = 0;

    tf2::Quaternion quat_tf;
    tf2::convert(stationTransform.transform.rotation, quat_tf);
    tf2::Matrix3x3 mat (quat_tf);
    double ignr, ignp, yaw;
    mat.getRPY(ignr, ignp, yaw);
    quat_tf.setRPY(0, 0, yaw);

    stationTransform.transform.rotation = tf2::toMsg(quat_tf);

    if (stationTransform.header.stamp != targetTransform->header.stamp)
    {
        *targetTransform = stationTransform;
        return true;
    }

    return false;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "target");

    ros::NodeHandle nh;

    // Parameters
    nh.param("target/tag_id", g_targetTag, 0);
    nh.param("target/target_frame_id", g_target_frame_id, std::string("target"));
    ROS_INFO("Searching for tag #%d", g_targetTag);



    // Publishers
    tf2_ros::TransformBroadcaster br;

    // Initializing the transform buffer
    tf2_ros::TransformListener tfListener(g_tfBuffer);



    geometry_msgs::TransformStamped targetTransform;
    geometry_msgs::TransformStamped constantTransform;

    ros::Rate rate(100.0);
    while(nh.ok() && !ComputeTarget(&targetTransform))
    {   // Waiting for the first transform
        rate.sleep();
    }

    constantTransform = targetTransform;

    while(nh.ok())
    {
        ++constantTransform.header.seq;
        constantTransform.header.stamp = ros::Time::now();

        if (ComputeTarget(&targetTransform))
            constantTransform.transform = targetTransform.transform;
        br.sendTransform(constantTransform);

        rate.sleep();
    }

    return 0;
}
