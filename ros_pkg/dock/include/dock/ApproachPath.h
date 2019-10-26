#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"


namespace dock
{


class ApproachPath
{
public:
    ApproachPath(const geometry_msgs::TransformStamped &);

    double getLength() const;
    double getAngleAfter(double dx) const;
    double getAttackAngle() const { return getAngleAfter(0); }
    double getAngleError() const;
    nav_msgs::Path getPath(double dx = 0.05) const;

private:
    double x1, y1, a;
    double robotTheta;
    std_msgs::Header pathHeader;

    double getY(double x) const;
    double getAngleAt(double x) const;
};


}// namespace dock

