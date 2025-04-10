#ifndef THRUSTER_MANAGER_THRUSTER_LINK_H
#define THRUSTER_MANAGER_THRUSTER_LINK_H

#include <string>
#include <algorithm>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <Eigen/Core>

namespace thruster_manager
{

struct ThrusterLink
{
  std::string frame;
  std::string joint;
  Eigen::Vector3d axis;

  explicit ThrusterLink(const std::string &frame, const std::string &joint, double x, double y, double z)
      : frame{frame}, joint{joint}, axis{x,y,z}
  {  }

  void write(geometry_msgs::msg::WrenchStamped &wrench, double f) const
  {
    wrench.header.frame_id = frame;
    wrench.wrench.force.x = -f*axis(0);
    wrench.wrench.force.y = -f*axis(1);
    wrench.wrench.force.z = -f*axis(2);
  }

  std::string topicName() const
  {
    auto name{this->frame};
    std::replace(name.begin(), name.end(), '/', '_');
    return name + "_wrench";
  }
};

}

#endif // THRUSTER_MANAGER_THRUSTER_LINK_H
