#include <thruster_manager/thruster_manager.h>
#include <thruster_manager/model_parser.h>

using namespace thruster_manager;
using namespace std;

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

inline void ThrusterManager::scale(Eigen::VectorXd &thrust) const
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
}


void ThrusterManager::scale(Eigen::VectorXd &thrust, size_t idx) const
{

  int max_idx{-1};
  auto max_thr{0.};
  for(uint t = 0; t < dofs; ++t)
  {
    const auto ratio = thrust(t) > 0 ? thrust(t)/fmax : thrust(t)/fmin;
    if(ratio > max_thr)
    {
      max_thr = ratio;
      max_idx = t;
    }
  }
  if(max_thr< 1. +1e-3)
    return;

  // thrust <- s*thrust + r*Z
  // thrust[idx] inchanged
  const auto t0{thrust(idx)};
  const auto z0{Z(idx)};
  // thrust[max_idx] saturated
  const auto tm{thrust(max_idx)};
  const auto zm{Z(max_idx)};
  const auto m = tm > 0 ? fmax : fmin;
  const auto det{1./(tm*z0-t0*zm)};
  const auto s{det*(m*z0 - t0*zm)};
  const auto r{det*(-m*t0 + t0*tm)};
  thrust = s*thrust + r*Z;
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
  const auto computeCost = [&](const Eigen::VectorXd &thrust)
  {
    if(insideDeadzone(thrust))
      return std::numeric_limits<double>::max();

    const auto signs{thrust.cwiseProduct(prev_thrust)};
    return (scaled_wrench-tam*thrust).norm()
        + cont_weight*std::count_if(kernel_thrusters.begin(), kernel_thrusters.end(),
                                    [&](uint idx){return signs(idx) < 0;});
  };

  Eigen::VectorXd best;
  double best_cost{std::numeric_limits<double>::max()};

  // test overall best thrust with opposed kernel
  for(auto s: {1,-1})
  {
    const auto ref{kernel_thrusters[0]};
    Eigen::VectorXd candidate{thrust + Z*(s*thrust(ref)-thrust(ref))/Z(ref)};
    if(const auto cost{computeCost(candidate)}; cost < best_cost)
    {
      best = candidate;
      best_cost = cost;
    }
  }

  for(auto ref: kernel_thrusters)
  {
    // force this @ +- deadzone
    for(auto d: {-deadzone, deadzone})
    {
      Eigen::VectorXd candidate{thrust + Z*(d-thrust(ref))/Z(ref)};
      scale(candidate, ref);

      if(const auto cost{computeCost(candidate)}; cost < best_cost)
      {
        best = candidate;
        best_cost = cost;
      }
    }
  }
  return prev_thrust = best;
}


