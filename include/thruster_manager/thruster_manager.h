#ifndef THRUSTER_MANAGER_THRUSTER_MANAGER_H
#define THRUSTER_MANAGER_THRUSTER_MANAGER_H

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <thruster_manager/thruster_link.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>

namespace thruster_manager
{

class ThrusterManager
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;

  ThrusterManager() {};

  inline auto use_tf() const
  {
    return tf_buffer != nullptr;
  }

  inline void setThrusterLimits(double fmin, double fmax, double deadzone)
  {
    assert(fmin <= 0);
    assert(fmax >= 0);
    assert(deadzone >= 0);
    this->fmin = fmin;
    this->fmax = fmax;
    this->deadzone = deadzone;
    computeKernel();
  }

  /// reads the robot_description parameter and returns the thruster links (frames / joints / pose wrt control frame)
  /// declares all parsing parameters in the given node
  template <class Node>
  std::vector<ThrusterLink> parseRobotDescription(Node *node, const std::string &control_frame)
  {
    static_assert (
    std::is_base_of_v<rclcpp::Node, Node> || std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, Node>,
        "ThrusterManager::parseRobotDescription: Node should be rclcpp::Node or rclcpp_lifecycle::LifecycleNode");
    assert (node != nullptr);

    this->control_frame = control_frame;

    const auto thrusters = node->template declare_parameter<std::vector<std::string>>("tam.thrusters",std::vector<std::string>{});
    const auto thruster_prefix = node->template declare_parameter<std::string>("tam.thruster_prefix", "");
    const auto use_gz =  node->declare_parameter("tam.use_gz_plugin", true);
    const auto use_tf = node->declare_parameter("tam.use_tf", false);

    setThrusterLimits(node->declare_parameter("tam.min_thrust", -40.),
                      node->declare_parameter("tam.max_thrust", 40.),
                      node->declare_parameter("tam.deadzone", 0.));

    cont_weight = node->declare_parameter("tam.continuity", .1);

    const auto links{parseRobotDescription(node,
                                           control_frame,
                                           thrusters,
                                           thruster_prefix,
                                           use_gz,
                                           use_tf)};

    // write actual thrusters as param
    // to be robust to empty vector<string> when dumping the parameters
    if(thrusters.empty() && node->has_parameter("tam.thrusters"))
    {
      std::vector<std::string> names;
      std::transform(links.begin(), links.end(), std::back_inserter(names), [](const ThrusterLink &link)
      {return link.joint;});
      node->set_parameter({"tam.thrusters", names});
    }

    return links;
  }

  /// reads the robot_description parameter and returns the thruster joints
  template <class Node>
  std::vector<ThrusterLink> parseRobotDescription(Node *node,
                                                  const std::string &control_frame,
                                                  const std::vector<std::string> &thrusters,
                                                  const std::string &thruster_prefix,
                                                  bool use_gz_plugin,
                                                  bool use_tf = false)
  {
    static_assert (
    std::is_base_of_v<rclcpp::Node, Node> || std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, Node>,
        "ThrusterManager::parseRobotDescription: Node should be rclcpp::Node or rclcpp_lifecycle::LifecycleNode");
    assert (node != nullptr);

	if(use_tf)
	{
	  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
	  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer, node);
	}

    return parseRobotDescription(node->template get_logger(),
                                 control_frame,
                                 thrusters,
                                 thruster_prefix,
                                 use_gz_plugin);
  }

  Eigen::VectorXd solveWrench(const Vector6d &wrench);

  // compute the max components of the wrench, assuming min/max thrust are non-0
  // useful for anti-windup in higher-level control
  Vector6d maxWrench() const;

private:

  Eigen::Matrix<double, 6, Eigen::Dynamic> tam;
  uint dofs{};
  Eigen::VectorXd prev_thrust;

  // thruster constraints
  double fmin{0}, fmax{0}, deadzone{0}, cont_weight{0.1};

  // scale this vector to [fmin,fmax]
  inline void scale(Eigen::VectorXd &thrust, bool ensure_deadzone = false) const;

  // kernel info
  std::vector<uint> kernel_thrusters;
  Eigen::VectorXd Z;
  void computeKernel();
  inline bool insideDeadzone(const Eigen::VectorXd &thrust) const
  {
    return std::any_of(kernel_thrusters.begin(),
                       kernel_thrusters.end(),
                       [&](uint t){return std::abs(thrust(t)) < deadzone - 1e-4;});
  }

  // TF listener if needed
  std::string control_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::vector<ThrusterLink> tf_thrusters;
  void updateTAM();

  // model parser
  std::vector<ThrusterLink> parseRobotDescription(const rclcpp::Logger &logger,
                                                  const std::string &control_frame,
                                                  const std::vector<std::string> &thrusters,
                                                  const std::string &thruster_prefix,
                                                  bool use_gz_plugin);
};

} // namespace thruster_manager


#endif
