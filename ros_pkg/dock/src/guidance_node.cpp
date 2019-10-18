#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/Path.h"
#include <base_local_planner/trajectory_planner_ros.h>
#include <string>
#include <cmath>
#include <mutex>


std::mutex g_goalMutex;
tf2_ros::Buffer g_tfBuffer;



void pathPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{

}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;


    // Initializing the transform buffer
    tf2_ros::TransformListener tfListener(g_tfBuffer);


    // Init publisher


    // Init subscibers



    ros::Rate rate(100.0);
    while(nh.ok())
    {

        rate.sleep();
    }

    return 0;
}
