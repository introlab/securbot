#include "dock/ApproachPath.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <algorithm>

namespace dock
{


ApproachPath::ApproachPath(const geometry_msgs::TransformStamped & robotT)
{
    x1 = robotT.transform.translation.x;
    y1 = robotT.transform.translation.y;
    a = y1 / x1 / x1;

    tf2::Quaternion quat_robot;
    tf2::convert(robotT.transform.rotation, quat_robot);
    tf2::Matrix3x3 mat (quat_robot);
    double i_r, i_p;
    mat.getRPY(i_r, i_p, robotTheta);

    pathHeader = robotT.header;
}


double ApproachPath::getY(double x)  const { return a * x *x; }


double ApproachPath::getAngleAt(double x) const
{
    return std::atan( 2 * a * x);
}


double ApproachPath::getLength() const
{
    return x1;
}


double ApproachPath::getAngleAfter(double dx) const
{
    return getAngleAt(x1 + dx);
}

double ApproachPath::getAngleError() const
{
    return robotTheta - getAttackAngle();
}


nav_msgs::Path ApproachPath::getPath(double dx) const
{
    nav_msgs::Path plan;
    plan.header = pathHeader;

    for (double x = 0; x > x1; x -= dx)
    {
        geometry_msgs::PoseStamped node;
        node.header = pathHeader;

        node.pose.position.x = x;
        node.pose.position.y = getY(x);
        node.pose.position.z = 0;

        tf2::Quaternion quat_node;
        quat_node.setRPY(0,0,getAngleAt(x));
        node.pose.orientation = tf2::toMsg(quat_node);

        plan.poses.push_back(node);
    }

    std::reverse(plan.poses.begin(), plan.poses.end());

    return plan;
}


}
