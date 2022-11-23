#pragma once

#include <ros/ros.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>

#include <ff_msgs/DockState.h>

#include <behaviortree_cpp_v3/action_node.h>

namespace astrobee
{

class UndockBtAction : public BT::ActionNodeBase
{
 public:
  UndockBtAction(const std::string &name,
		 const BT::NodeConfiguration &conf);
  UndockBtAction() = delete;

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("robot")
	};
  }
 protected:
  bool on_tick();
  void reset();
  void dockStateCallback(const ff_msgs::DockState& msg);
  BT::NodeStatus check_future();
  
  std::string state_topic_;
  std::string command_topic_;
  ros::Subscriber dock_state_sub_;
  ros::Publisher cmd_pub_;
  int berth_;

  bool docked_;
  bool undocked_;
  bool got_state_;

  // Async stuff
  bool is_undocking_;
  std::future<bool> future_result_;
  
  std::shared_ptr<ros::NodeHandle> node_;

  // Parameter
  std::string robot_;
};

} // namespace astrobee
