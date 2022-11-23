
#include <astrobee_task_planning_examples/plugins/actions/dock_bt_action.h>


DockBtAction::DockBtAction(const std::string &_name,
			   const BT::NodeConfiguration &_conf)
  : BT::ActionNodeBase(_name, _conf),
{
  node_ = config().blackboard->get<ros::NodeHandle>("node");
  reset();

  return true;
}

/**
 * @function reset
 */    
void DockBtAction::reset()
{
  berth_ = 1;
  docked_ = false;
  undocked_ = false;
  got_state_ = false;
}

bool DockBtAction::on_tick()
{
  reset();
  
  if(!getInput("robot", robot_))
    return false;

  state_topic_ = "/" + robot_ + "/beh/dock/state";
  dock_state_sub_ = node_->subscribe(state_topic_, 10, &DockBtAction::dockStateCallback, this);

  command_topic_ = "/" + robot_ + "/command";
  return true;
}

    
BT::NodeStatus DockBtAction::tick()
{
  if (!is_undocking)
  {
    future_result_ = std::async(std::launch::async, [this]() {

						      // Read parameters
						      if(!on_tick())
							return false;
						      
						      //
						      ff_msgs::CommandStamped cmd;
						      cmd.header.stamp = ros::Time::now();
						      cmd.subsys_name = robot_;
						      
						      cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_DOCK;
						      cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_DOCK;
						      
						      // Dock has one argument
						      cmd.args.resize(1);
						      cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
						      cmd.args[0].i = berth_;

						      cmd_pub_.publish(cmd);
						      
						      ROS_INFO_STREAM("DockBtAction -- Docking " << robot_ << " at berth: " << berth_);

						      while (!got_state_ && !docked_)
						      {
							ros::Duration(0.1).sleep();
							ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- waiting for state " << robot_name_);
						      }
						      if (docked_)
						      {
							ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- Already docked, returning...");
							return true;
						      }

						      ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- sending dock command...");
						      docked_ = false;
						      got_state_ = false;
						      command_pub_.publish(cmd);
						      ros::Duration(2.0).sleep();
						      while (!docked_)
						      {
							ros::Duration(1.0).sleep();
							ROS_DEBUG_STREAM("[DockState::execute()] docking " << robot_name_);
						      }
						      ROS_INFO_STREAM("[DockState::execute()] docking complete...");
						      return true;
						      
						    });
    is_undocking_ = true;

  } // if !is_undocking

  return check_future();
  
}




void DockBtAction::dockStateCallback(const ff_msgs::DockState& msg)
{
  ROS_INFO_STREAM_THROTTLE(1, "[DockState::dockStateCallback()]");
  if (!got_state_)
  {
    docked_ = (msg.state == ff_msgs::DockState::DOCKED);
    ROS_INFO_STREAM_THROTTLE(1, "[DockState::dockStateCallback()] -- docked: " << (int)docked_);
  }
  undocked_ = (msg.state == ff_msgs::DockState::UNDOCKED);
  got_state_ = true;
  ROS_INFO_STREAM_THROTTLE(1, "[DockState::dockStateCallback()] -- undocked: " << (int)undocked_);

