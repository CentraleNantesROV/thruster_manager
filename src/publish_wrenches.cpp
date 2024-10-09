#include <thruster_manager/thruster_manager.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace thruster_manager
{



rcl_interfaces::msg::ParameterDescriptor description(const std::string &des)
{
  return rcl_interfaces::msg::ParameterDescriptor().set__description(des);
}

class Publisher : public rclcpp::Node
{

  using Float64 = std_msgs::msg::Float64;
  using JointState = sensor_msgs::msg::JointState;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;

public:
  explicit Publisher() : Node("wrench_publisher")
  {
    const auto control_frame{declare_parameter<std::string>("control_frame", "base_link")};
    const auto sub_gz{declare_parameter("use_gz_topics", true,  description("If the command should be subscribed through gz topics (as opposed to joint thrusts)"))};

    ThrusterManager tm;
    links = tm.parseRobotDescription(this, control_frame, {}, {}, true);
    dofs = links.size();
    thrusts.resize(dofs, 0);

    if(sub_gz)
    {
      for(size_t thr=0; thr<dofs; ++thr)
      {
        cmd_gz_sub.push_back(create_subscription<Float64>("cmd_" + links[thr].joint, 10, [=](Float64::ConstSharedPtr thrust)
        {thrusts[thr] = thrust->data;}));
      }
    }
    else
    {
      cmd_js_sub = create_subscription<JointState>("cmd_thrust", 10, [&](JointState::ConstSharedPtr js){parseJS(*js);});
    }


    std::transform(links.begin(), links.end(), std::back_inserter(wrench_pub), [this](const auto &link)
    {
      return create_publisher<WrenchStamped>(link.topicName(), 10);
    });

    pub_timer = create_wall_timer(100ms, [&](){publishWrenches();});
  }


  size_t dofs;
  std::vector<ThrusterLink> links;
  std::vector<double> thrusts;


  // if we subscribe to joint states
  rclcpp::Subscription<JointState>::SharedPtr cmd_js_sub;

  // if we subscribes to Gz topics
  std::vector<rclcpp::Subscription<Float64>::SharedPtr> cmd_gz_sub;

  // we publish wrenches anyway
  rclcpp::TimerBase::SharedPtr pub_timer;
  WrenchStamped wrench;
  std::vector<rclcpp::Publisher<WrenchStamped>::SharedPtr> wrench_pub;

  void parseJS(const JointState &js)
  {
    size_t idx{};
    for(const auto &name: js.name)
    {
      const auto link{std::find_if(links.begin(), links.end(), [&](const auto &link){return link.joint == name;})};
      if(link != links.end())
        thrusts[std::distance(links.begin(), link)] = js.effort[idx];
      idx++;
    }
  }

  void publishWrenches()
  {
    wrench.header.stamp = get_clock()->now();

    for(size_t thr = 0; thr < dofs; ++thr)
    {
      links[thr].write(wrench, thrusts[thr]);
      wrench_pub[thr]->publish(wrench);
    }
  }



};
}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thruster_manager::Publisher>());
  rclcpp::shutdown();
  return 0;
}

