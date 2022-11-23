#pragma once

#include <ff_msgs/Command.h>


class DockBtAction : public BT::ActionNodeBase
{
 public:
  DockBtAction(const std::string &name,
	       const BT::NodeConfiguration &conf);
  DockBtAction() = delete;

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providePorts()
  {
    return {
      BT::InputPort<std::string>("robot")
	};
  }
 protected:
  std::string state_topic_;
  std::string command_topic_;
  ros::Subscriber dock_state_sub_;
  ros::Publisher cmd_pub_;
  int berth_;
};
