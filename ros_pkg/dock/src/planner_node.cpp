#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include <string>
#include <cmath>


double g_dx;
tf2_ros::Buffer g_tfBuffer;
ros::Publisher g_pathPublisher;
std::string g_target_frame_id;
std::string g_base_frame_id;

void ComputeTrajectory()
{
    // Evaluating new target
    static ros::Time lastTimestamp;
    geometry_msgs::TransformStamped robotTransform;
    try {
        robotTransform = g_tfBuffer.lookupTransform(
                g_target_frame_id,
                g_base_frame_id,
                ros::Time(0));
    }
    catch (...) { return; }

    if (robotTransform.header.stamp == lastTimestamp) // Trajectory has not changed
        return;
    lastTimestamp = robotTransform.header.stamp;



    // Compute path
    nav_msgs::Path plan;
    double dx = g_dx,
           x1 = robotTransform.transform.translation.x,
           y1 = robotTransform.transform.translation.y,
           a = y1 / x1 / x1;

    for(double x = 0; x > x1; x -= dx)
    {
        geometry_msgs::PoseStamped node;
        node.header = robotTransform.header;

        double y = a * x * x;

        node.pose.position.x = x;
        node.pose.position.y = y;
        node.pose.position.z = 0;

        double theta = std::atan(2 * a * x);

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, theta);

        node.pose.orientation = tf2::toMsg(quat_tf);

        plan.poses.push_back(node);
    }

    plan.header = robotTransform.header;

    g_pathPublisher.publish(plan);
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;


    // Parameters
    nh.param("target/target_frame_id", g_target_frame_id, std::string("target"));
    nh.param("target/base_frame_id", g_base_frame_id, std::string("base_footprint"));
    nh.param("target/dx", g_dx, 0.05);


    // Initializing the transform buffer
    tf2_ros::TransformListener tfListener(g_tfBuffer);


    // Init publisher
    g_pathPublisher = nh.advertise<nav_msgs::Path>("approach_plan", 10);


    ros::Rate rate(100.0);
    while(nh.ok())
    {
        ComputeTrajectory();

        rate.sleep();
    }

    return 0;
}
