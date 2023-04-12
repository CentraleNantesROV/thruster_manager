#include <thruster_manager/thruster_manager.h>
#include <thruster_manager/model_parser.h>

using namespace thruster_manager;
using namespace std;

std::vector<ThrusterLink> ThrusterManager::parseRobotDescription(rclcpp::Node* node,
                                                                const string &control_frame)
{
  assert (node != nullptr);
  const auto thrusters = node->declare_parameter<vector<string>>("tam.thrusters",vector<string>{});
  const auto thruster_prefix = node->declare_parameter<string>("tam.thruster_prefix", "");
  const auto use_gz = node->declare_parameter("tam.use_gz_plugin", true);

  setThrusterLimits(node->declare_parameter("tam.min_thrust", -40.),
                    node->declare_parameter("tam.max_thrust", 40.),
                    node->declare_parameter("tam.deadzone", 0.));

  return parseRobotDescription(node,
                               control_frame,
                               thrusters,
                               thruster_prefix,
                               use_gz);
}

void ThrusterManager::computeKernel()
{
  if(deadzone > 0 && dofs > 0)
  {
    // identify kernel dimension and thrusters belonging to the kernel, if any
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(tam.transpose());
    Z = Eigen::MatrixXd(qr.matrixQ()).block(0, qr.rank(), dofs, dofs-qr.rank());
    prev_thrust.resize(dofs);
    prev_thrust.setZero();

    if(Z.cols() != 0)
    {
      // register which thrusters may be played with in the kernel
      for(uint t = 0; t < dofs; ++t)
      {
        if(Z.row(t).norm() > 1e-6)
          kernel_thrusters.push_back(t);
      }
    }
  }
}

std::vector<ThrusterLink> ThrusterManager::parseRobotDescription(rclcpp::Node* node,
                                                                const std::string &control_frame,
                                                                const std::vector<std::string> &thrusters,
                                                                const std::string &thruster_prefix,
                                                                bool use_gz_plugin)
{
  assert (node != nullptr);
  const auto &logger{node->get_logger()};

  auto model{ModelParser(control_frame)};
  if(!model.valid())
    return {};

  if(!model.hasControlFrame())
  {
    // cannot get the model anyway
    RCLCPP_ERROR(logger, "Control frame '%s' cannot be found in robot description",
                 control_frame.c_str());
    return {};
  }

  const auto joints{model.extractJoints(thrusters, thruster_prefix, use_gz_plugin)};
  if(joints.empty())
  {
    RCLCPP_ERROR(logger, "Thruster manager: no thruster joints were found");
    return {};
  }

  // write transform into tam
  dofs = joints.size();
  tam.resize(6, dofs);

  std::vector<ThrusterLink> links;
  uint col{0};
  for(const auto &[name,joint]: joints)
  {
    RCLCPP_INFO(logger, "Found thruster %s", name.c_str());
    tam.col(col++) = model.thrusterMapping(joint);    
    links.push_back(model.thrusterLink(joint));
  }

  computeKernel();

  return links;
}

ThrusterManager::Vector6d ThrusterManager::maxWrench() const
{
  Vector6d wrench;
  wrench.setZero();
  const auto thrust{std::max(fmax, -fmin)};

  for(size_t dir = 0; dir < 6; ++dir)
  {
    for(uint thr = 0; thr < dofs; ++thr)
      wrench(dir) += thrust * std::abs(tam(dir, thr));
  }
  return wrench;
}


Eigen::VectorXd ThrusterManager::solveWrench(const Vector6d &wrench, Limits limits)
{
  static const auto tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  Eigen::VectorXd thrust{tamInv*wrench};

  if(limits == Limits::SCALE)
  {
  if(const auto scale{scaleFactor(thrust)}; scale > 1)
    thrust /= scale;
  }
  else if(limits == Limits::SATURATE)
  {
    saturate(thrust);
  }

  if(!kernel_thrusters.empty())
  {
    // try to play inside the kernel to avoid deadzones
    auto candidate = thrust;
    std::vector<uint> done;
    static std::vector<double> margin(kernel_thrusters.size());
    static const auto unit{Eigen::VectorXd::Ones(Z.cols())};
    uint valid = countValid(candidate);

    while(valid != kernel_thrusters.size())
    {
      // find worst violated deadzone
      std::transform(kernel_thrusters.begin(), kernel_thrusters.end(),
                     margin.begin(),
                     [&](int thr){return std::abs(candidate(thr)) - deadzone;});
      const auto worst{std::min_element(margin.begin(), margin.end())};
      if(*worst >= -1e-3)
        break;

      const auto idx{kernel_thrusters[std::distance(margin.begin(), worst)]};

      // not be smart
      if(std::find(done.begin(), done.end(), idx) != done.end())
        break;
      done.push_back(idx);

      // try to keep same sign as previous iteration, avoid chattering
      const auto offset = (prev_thrust(idx) > 0 ? deadzone : -deadzone) - candidate(idx);
      // update candidate anyway
      candidate += Z * (offset / Z.row(idx).sum()) * unit;
      // save if better candidate
      if(const auto cand_valid{countValid(candidate)}; cand_valid > valid)
      {
        thrust = candidate;
        valid = cand_valid;
      }
    }
    prev_thrust = thrust;
  }

  return thrust;
}


