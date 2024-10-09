#include <thruster_manager/thruster_manager_node.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>

using namespace thruster_manager;

using geometry_msgs::msg::WrenchStamped;

rcl_interfaces::msg::ParameterDescriptor description(const std::string &des)
{
  return rcl_interfaces::msg::ParameterDescriptor().set__description(des);
}

ThrusterManagerNode::ThrusterManagerNode(rclcpp::NodeOptions options)
  : Node("thruster_manager", options)
{

  const auto pub_js{declare_parameter("publish_joint_state", true,  description("If the command should be published as joint state"))};
  const auto pub_gz{declare_parameter("publish_gz_command", true, description("If the command should be published as Float64 (Gazebo thruster plugin)"))};
  const auto sub_stamped{declare_parameter("subscribe_stamped", false,
                                           description("If the node should expect WrenchStamped messages instead of Wrench"))};
  const auto control_frame{declare_parameter<std::string>("control_frame", "base_link")};

  // joint names are stored here anyway to sync joint names and indices
  const auto links{allocator.parseRobotDescription(this, control_frame)};
  std::transform(links.begin(), links.end(), std::back_inserter(cmd.name), [](const auto &link){return link.joint;});
  dofs = cmd.name.size();
  cmd.effort.resize(dofs);

  if(pub_js)
    cmd_js_pub = create_publisher<JointState>("cmd_thrust", 5);

  if(pub_gz)
  {
    for(const auto &name: cmd.name)
    {
      const auto topic{"cmd_" + name};
      cmd_gz_pub.push_back(create_publisher<Float64>(topic,5));
    }
  }

  if(sub_stamped)
  {
    wrench_sub = create_subscription<WrenchStamped>("wrench", 5, [&](const WrenchStamped::ConstSharedPtr msg)
    {
      solve(msg->wrench);
    });
  }
  else
  {
    wrench_sub = create_subscription<Wrench>("wrench", 5, [&](const Wrench::ConstSharedPtr msg)
    {
      solve(*msg);
    });
  }
}


void ThrusterManagerNode::solve(const Wrench &wrench)
{
  ThrusterManager::Vector6d F;
  F(0) = wrench.force.x;
  F(1) = wrench.force.y;
  F(2) = wrench.force.z;
  F(3) = wrench.torque.x;
  F(4) = wrench.torque.y;
  F(5) = wrench.torque.z;

  const auto thrusts{allocator.solveWrench(F)};

  if(cmd_js_pub)
  {
    std::copy(thrusts.data(), thrusts.data()+dofs, cmd.effort.begin());
    cmd.header.stamp = get_clock()->now();
    cmd_js_pub->publish(cmd);
  }

  if(!cmd_gz_pub.empty())
  {
    static Float64 cmd_gz;
    for(size_t i = 0; i < dofs; ++i)
    {
      cmd_gz.data = thrusts[i];
      cmd_gz_pub[i]->publish(cmd_gz);
    }
  }
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(thruster_manager::ThrusterManagerNode)
