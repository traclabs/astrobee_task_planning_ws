
#include <astrobee_task_planning_examples/plugins/actions/undock_bt_action.h>

namespace astrobee
{

  /**
   * @function UndockBtAction
   * @brief Constructor
   */
UndockBtAction::UndockBtAction(const std::string &_name,
			   const BT::NodeConfiguration &_conf) :
  BT::ActionNodeBase(_name, _conf)
{
  node_ = config().blackboard->get< std::shared_ptr<ros::NodeHandle> >("node");
  reset();
}

/**
 * @function reset
 */    
void UndockBtAction::reset()
{
  berth_ = 1;
  docked_ = false;
  undocked_ = false;
  got_state_ = false;

  is_undocking_ = false;
}

bool UndockBtAction::on_tick()
{
  reset();
  
  if(!getInput("robot", robot_))
    return false;

  state_topic_ = "/" + robot_ + "/beh/dock/state";
  dock_state_sub_ = node_->subscribe(state_topic_, 10, &UndockBtAction::dockStateCallback, this);

  command_topic_ = "/" + robot_ + "/command";
  cmd_pub_ = node_->advertise<ff_msgs::CommandStamped>(command_topic_, 10);
  
  return true;
}

    
BT::NodeStatus UndockBtAction::tick()
{
  if (!is_undocking_)
  {
    future_result_ = std::async(std::launch::async, [this]() {

						      // Read parameters
						      if(!on_tick())
							return false;
						      
						      //
						      ff_msgs::CommandStamped cmd;
						      cmd.header.stamp = ros::Time::now();
						      cmd.subsys_name = robot_;
						      
						      cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_UNDOCK;
						      cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_UNDOCK;
						      
						      // Dock has one argument
						      cmd.args.resize(1);
						      cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
						      cmd.args[0].i = berth_;

						      cmd_pub_.publish(cmd);
						      
						      ROS_INFO_STREAM("DockBtAction -- Docking " << robot_ << " at berth: " << berth_);

						      while (!got_state_ && !docked_)
						      {
							ros::Duration(0.1).sleep();
							ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- waiting for state " << robot_);
						      }
						      if (!docked_)
						      {
							ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- Already undocked, returning...");
							return true;
						      }

						      ROS_INFO_STREAM_THROTTLE(1, "DockBtAction -- sending undock command...");
						      undocked_ = false;
						      got_state_ = false;
						      cmd_pub_.publish(cmd);
						      ros::Duration(2.0).sleep();
						      while (!docked_)
						      {
							ros::Duration(1.0).sleep();
							ROS_DEBUG_STREAM("[DockState::execute()] docking " << robot_);
						      }
						      ROS_INFO_STREAM("[DockState::execute()] docking complete...");
						      return true;
						      
						    });
    is_undocking_ = true;

  } // if !is_undocking

  return check_future();
  
}


/**
 * @function dockStateCallback
 */
void UndockBtAction::dockStateCallback(const ff_msgs::DockState& msg)
{
  ROS_INFO_STREAM_THROTTLE(1, "UndockBtAction");
  if (!got_state_)
  {
    docked_ = (msg.state == ff_msgs::DockState::DOCKED);
    ROS_INFO_STREAM_THROTTLE(1, "[DockState::dockStateCallback()] -- docked: " << (int)docked_);
  }
  undocked_ = (msg.state == ff_msgs::DockState::UNDOCKED);
  got_state_ = true;
  ROS_INFO_STREAM_THROTTLE(1, "[DockState::dockStateCallback()] -- undocked: " << (int)undocked_);
}

/**
 *
 */
BT::NodeStatus UndockBtAction::check_future()
{
  auto timeout = std::chrono::milliseconds(10);
  if(future_result_.valid() && future_result_.wait_for(timeout) == std::future_status::ready)
  {
    is_undocking_ = false;
    if(future_result_.get() == true)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;  
}

/**
 * @brief Stop execution
 */  
void UndockBtAction::halt()
{
  is_undocking_ = false;
  setStatus(BT::NodeStatus::IDLE);
}

} // namespace astrobee

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<astrobee::UndockBtAction>("Undock");
}
