#include <thruster_manager/thruster_manager.h>
#include <thruster_manager/model_parser.h>

using namespace thruster_manager;
using namespace std;

int sgn(double val)
{
  return (0. < val) - (val < 0.);
}

void ThrusterManager::computeKernel()
{
  if(deadzone > 0 && dofs > 0)
  {
    // identify kernel dimension and thrusters belonging to the kernel, if any
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(tam.transpose());
    const auto Zfull{Eigen::MatrixXd(qr.matrixQ()).block(0, qr.rank(), dofs, dofs-qr.rank())};

    prev_thrust.resize(dofs);
    prev_thrust.setZero();

    if(Zfull.cols() != 0)
    {
      // Kernel of dim. 1, limited to horizontal thrusters
      Z.resize(dofs);
      // register which horizontal thrusters may be played with in the kernel
      for(uint t = 0; t < dofs; ++t)
      {
        Z(t) = Zfull.row(t).sum();
        if((std::abs(tam(2,t)) <  1e-3)     // horizontal
           && std::abs(Z(t)) > 1e-6)        // vectored
          kernel_thrusters.push_back(t);
      }
    }
  }
}

std::vector<ThrusterLink> ThrusterManager::parseRobotDescription(const rclcpp::Logger &logger,
                                                                 const std::string &control_frame,
                                                                 const std::vector<std::string> &thrusters,
                                                                 const std::string &thruster_prefix,
                                                                 bool use_gz_plugin)
{
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

inline void ThrusterManager::scale(Eigen::VectorXd &thrust, bool ensure_deadzone) const
{
  auto scale{1.};
  if(fmin == 0 || fmax == 0)
    return;

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
  if(scale > 1.)
    thrust /= scale;
  if(ensure_deadzone)
  {
    for(auto &t: thrust)
    {
      if(std::abs(t) < deadzone)
        t = sgn(t)*deadzone;
    }
  }
}

Eigen::VectorXd ThrusterManager::solveWrench(const Vector6d &wrench)
{
  static const Eigen::MatrixXd tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  Eigen::VectorXd thrust{tamInv*wrench};

  if(fmin == 0 || fmax == 0)
  {
    // no way to scale this
    return thrust;
  }

  scale(thrust);
  if(kernel_thrusters.empty())
  {
    // no kernel to adjust
    return thrust;
  }

  // target wrench
  const auto scaled_wrench{tam * thrust};

  // try to play inside the kernel to avoid deadzones
  Eigen::VectorXd best = thrust;
  double best_cost{std::numeric_limits<double>::max()};

  const auto checkCandidate = [&](Eigen::VectorXd thrust)
  {
    if(insideDeadzone(thrust))
      return;

    scale(thrust, true);

    const auto signs{thrust.cwiseProduct(prev_thrust)};
    const auto cost{(scaled_wrench-tam*thrust).norm()
          + cont_weight*std::count_if(signs.begin(),signs.end(), [](auto s)
      {return s < 0;})};
    if(cost < best_cost)
    {
      best_cost = cost;
      best = thrust;
    }
  };

  // consider base thrust
  checkCandidate(thrust);

  // all the deadzones we might want to use
  for(auto t: kernel_thrusters)
  {
    for(auto s: {-1, 1})
    {
      const auto offset{(s*sgn(thrust(t))*deadzone - thrust(t))/Z(t)};
      checkCandidate(thrust + offset*Z);
    }
  }
  return prev_thrust = best;
}


