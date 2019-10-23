#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <string>
#include <mutex>


std::mutex g_goalMutex;
tf2_ros::Buffer g_tfBuffer;


void GoalCallback(const nav_msgs::Path::ConstPtr& msg)
{
    std::scoped_lock(g_goalMutex);

    ROS_INFO("Updating plan");

}

void OutputGuidance()
{
    std::scoped_lock(g_goalMutex);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "guidance");
    ros::NodeHandle nh;


    // Initializing the transform buffer
    tf2_ros::TransformListener tfListener(g_tfBuffer);

    // Init subscribers
    ros::Subscriber goalSubscriber = nh.subscribe("approach_plan", 100, GoalCallback);

    // Init publisher


    // Init local planner
    costmap_2d::Costmap2DROS costmap("guidance_costmap", g_tfBuffer);
    base_local_planner::TrajectoryPlannerROS localPlanner;
    localPlanner.initialize("my_trajectory_planner", &g_tfBuffer, &costmap);


    ros::Rate rate(100.0);
    while(nh.ok())
    {
        OutputGuidance();

        rate.sleep();
    }

    return 0;
}
