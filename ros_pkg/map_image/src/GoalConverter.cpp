#include "map_image_generator/GoalConverter.h"
#include "map_image_generator/utils.h"

using namespace map_image_generator;

GoalConverter::GoalConverter(const Parameters& parameters,
    ros::NodeHandle& nodeHandle) :
    m_parameters(parameters),
    m_nodeHandle(nodeHandle)
{
    m_mapImageGoalSubscriber = m_nodeHandle.subscribe("map_image_goal", 1000,
        &GoalConverter::mapImageGoalCallback, this);
    m_goalPublisher = m_nodeHandle.advertise<geometry_msgs::PoseStamped>("output_goal", 50);
}

GoalConverter::~GoalConverter()
{
}

void GoalConverter::mapImageGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& mapImageGoal)
{
    m_goalPublisher.publish(convertMapImageToMap(m_parameters, *mapImageGoal));
}
