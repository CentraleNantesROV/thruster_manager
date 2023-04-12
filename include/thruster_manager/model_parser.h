#ifndef THRUSTER_MANAGER_MODEL_PARSER_H
#define THRUSTER_MANAGER_MODEL_PARSER_H

#include <urdf/model.h>
#include "thruster_link.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>

namespace thruster_manager
{

struct ModelParser
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  urdf::Model model;
  KDL::Tree tree;
  tinyxml2::XMLDocument model_xml;
  std::string control_frame;

  ModelParser(const std::string &control_frame) : control_frame{control_frame}
  {
    // use a unique node name for this node, only used to get the robot_description
    const auto rsp_node(std::make_shared<rclcpp::Node>("thruster_manager_rsp_" + std::to_string(getpid())));

    const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
        (rsp_node, "robot_state_publisher");

    rsp_param_srv->wait_for_service();

    if(!rsp_param_srv->has_parameter("robot_description"))
    {
      // cannot get the model anyway
      RCLCPP_ERROR(rsp_node->get_logger(), "cannot get model in namespace %s", rsp_node->get_namespace());
      return;
    }

    const auto xml{rsp_param_srv->get_parameter<std::string>("robot_description")};

    model_xml.Parse(xml.c_str());
    model.initString(xml);
    // remove base link inertia to disable KDL warning
    model.root_link_->inertial.reset();
    kdl_parser::treeFromUrdfModel(model, tree);

  }

  inline bool valid() const
  {
    return !model.links_.empty();
  }

  inline bool hasControlFrame() const
  {
    return model.links_.find(control_frame) != model.links_.end();
  }

  auto extractJoints(std::vector<std::string> thrusters,
                     const std::string &thruster_prefix,
                     bool use_gz_plugin) const
  {
    std::vector<std::pair<std::string, urdf::JointSharedPtr>> joints;

    // if no hint, just use all revolute / continuous joints
    if(thrusters.empty() && thruster_prefix.empty() && !use_gz_plugin)
    {
      std::copy_if(model.joints_.begin(), model.joints_.end(), std::back_inserter(joints),
                   [&](const auto &joint)
      {return joint.second->type == urdf::Joint::REVOLUTE
            || joint.second->type == urdf::Joint::CONTINUOUS;});
    }
    else if(use_gz_plugin)
    {
      // populate thrusters with names from gz Thruster plugins
      auto root{model_xml.RootElement()};
      for(auto gz = root->FirstChildElement("gazebo");
          gz != nullptr;
          gz = gz->NextSiblingElement("gazebo"))
      {
        for(auto plugin = gz->FirstChildElement("plugin");
            plugin != nullptr;
            plugin = plugin->NextSiblingElement("plugin"))
        {
          const auto name{plugin->Attribute("name")};
          if(std::string(name).find("systems::Thruster") != std::string::npos)
          {
            auto joint{plugin->FirstChildElement("joint_name")};
            if(joint)
              thrusters.push_back(joint->GetText());
          }
        }
      }
    }

    std::copy_if(model.joints_.begin(), model.joints_.end(), std::back_inserter(joints),
                 [&](const auto &joint)
    {
      if(std::find(thrusters.begin(), thrusters.end(), joint.first) != thrusters.end())
        return true;
      if(thruster_prefix.empty())
        return false;
      return joint.first.find(thruster_prefix) == 0;});

    return joints;
  }

  static inline ThrusterLink thrusterLink(urdf::JointSharedPtr joint)
  {
    return ThrusterLink(joint->child_link_name, joint->name, joint->axis.x, joint->axis.y, joint->axis.z);
  }

  Vector6d thrusterMapping(urdf::JointSharedPtr joint)
  {
    // extract chain
    KDL::Chain chain;
    tree.getChain(control_frame, joint->child_link_name, chain);

    const auto n{chain.getNrOfJoints()};

    auto solver{KDL::ChainFkSolverPos_recursive(chain)};
    KDL::Frame thruster;
    solver.JntToCart(KDL::JntArray(n), thruster);

    Eigen::Matrix<double, 6, 1> col;
    // convert thruster pose to mapping through Eigen types
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    for(uint i = 0; i < 3; ++i)
      p(i) = thruster.p[i];
    for(uint i = 0; i < 9; ++i)
      R(i/3,i%3) = thruster.M.data[i];

    col.head<3>() = R*Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
    col.tail<3>() = p.cross(col.head<3>());
    return col;
  }
};

}

#endif // THRUSTER_MANAGER_MODEL_PARSER_H
