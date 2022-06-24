#ifndef THRUSTER_MANAGER_THRUSTER_MANAGER_H
#define THRUSTER_MANAGER_THRUSTER_MANAGER_H

#include <rclcpp/node.hpp>
#include <Eigen/Core>

namespace thruster_manager
{

class ThrusterManager
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  ThrusterManager() {};

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

  // reads the robot_description parameter and returns the thruster joints
  // declares all parsing parameters in the given node
  std::vector<std::string> parseRobotDescription(rclcpp::Node *node, const std::string &control_frame);

  // reads the robot_description parameter and returns the thruster joints
  std::vector<std::string> parseRobotDescription(rclcpp::Node *node,
                             const std::string &control_frame,
                             const std::vector<std::string> &thrusters,
                             const std::string &thruster_prefix,
                             bool use_gz_plugin);

  Eigen::VectorXd solveWrench(const Vector6d &wrench);

  // compute the max components of the wrench, assuming min/max thrust are non-0
  // useful for anti-windup in higher-level control
  Vector6d maxWrench() const;

private:

  Eigen::Matrix<double, 6, Eigen::Dynamic> tam;
  uint dofs{};
  Eigen::VectorXd prev_thrust;

  // thruster constraints
  double fmin{0}, fmax{0}, deadzone{0};

  inline double scaleFactor(const Eigen::VectorXd &thrust) const
  {
    auto scale{1.};
    if(fmin == 0 || fmax == 0)
      return scale;

#if EIGEN_VERSION_AT_LEAST(3,4,0)
    for(auto &thr: thrust)
      scale = std::max(scale, thr > 0 ? thr/fmax : thr/fmin);
#else
    for(uint t = 0; t < dofs; ++t)
    {
      auto &thr{thrust(t)};
      scale = std::max(scale, thr > 0 ? thr/fmax : thr/fmin);
    }
#endif
    return scale;
  }

  // kernel info
  std::vector<uint> kernel_thrusters;
  Eigen::MatrixXd Z;

  void computeKernel();

  inline uint countValid(const Eigen::VectorXd &thrust)
  {
    return std::count_if(kernel_thrusters.begin(),
                         kernel_thrusters.end(),
                         [&](uint t){return std::abs(thrust(t)) >= deadzone - 1e-6;});
  }
};

} // namespace thruster_manager


#endif
