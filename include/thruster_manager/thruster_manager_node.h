#ifndef THRUSTER_MANAGER_THRUSTER_MANAGER_NODE_H
#define THRUSTER_MANAGER_THRUSTER_MANAGER_NODE_H

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <thruster_manager/thruster_manager.h>

namespace thruster_manager
{

class ThrusterManagerNode : public rclcpp::Node
{
  using Float64 = std_msgs::msg::Float64;
  using JointState = sensor_msgs::msg::JointState;
  using Wrench = geometry_msgs::msg::Wrench;

public:
  ThrusterManagerNode(rclcpp::NodeOptions options);

private:

  ThrusterManager allocator;
  size_t dofs;

  rclcpp::SubscriptionBase::SharedPtr wrench_sub;
  void solve(const Wrench &wrench);

  // if we publish output as joint states
  sensor_msgs::msg::JointState cmd;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_js_pub;

  // if we publish output as Float64 (Gazebo thruster plugin)
  std::vector<rclcpp::Publisher<Float64>::SharedPtr> cmd_gz_pub;
};
}


#endif
