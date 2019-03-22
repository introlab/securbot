#ifndef GOAL_TRANSLATOR_H
#define GOAL_TRANSLATOR_H

#include "map_image_generator/Parameters.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

namespace map_image_generator
{
    class GoalConverter
    {
        const Parameters& m_parameters;
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_mapImageGoalSubscriber;
        ros::Publisher m_goalPublisher;
        
    public:
        GoalConverter(const Parameters& parameters, ros::NodeHandle& nodeHandle);
        virtual ~GoalConverter();

    private:
        void mapImageGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& mapImageGoal);
    };
}
#endif
