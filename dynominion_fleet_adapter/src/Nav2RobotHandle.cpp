#include <dynominion_fleet_adapter/Nav2RobotHandle.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>

Nav2RobotHandle::Nav2RobotHandle(
  const std::string& name,
  const rclcpp::Node::SharedPtr& node)
: name_(name),
  node_(node)
{
  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/" + name_ + "/navigate_to_pose");

  RCLCPP_INFO(node_->get_logger(), 
    "[Nav2RobotHandle][%s] Initialized (Industrial decoupled mode).", name_.c_str());
}

void Nav2RobotHandle::set_update_handle(const std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle>& update_handle)
{
  update_handle_ = update_handle;
}

bool Nav2RobotHandle::is_ready()
{
  // Readiness is determined by whether a valid localization pose has been received.
  return is_localized_;
}

void Nav2RobotHandle::update_state(const Eigen::Vector3d& pose, double battery, bool localized)
{
  cached_pose_ = pose;
  cached_battery_ = battery;
  is_localized_ = localized;
}

rmf_fleet_adapter::agv::EasyFullControl::RobotState Nav2RobotHandle::get_state()
{
  return rmf_fleet_adapter::agv::EasyFullControl::RobotState(
    level_name_, cached_pose_, cached_battery_);
}

void Nav2RobotHandle::navigate(
  rmf_fleet_adapter::agv::EasyFullControl::Destination destination,
  rmf_fleet_adapter::agv::EasyFullControl::CommandExecution execution)
{
  active_execution_ = std::make_unique<rmf_fleet_adapter::agv::EasyFullControl::CommandExecution>(std::move(execution));

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  
  Eigen::Vector3d pos = destination.position();
  goal_msg.pose.pose.position.x = pos.x();
  goal_msg.pose.pose.position.y = pos.y();
  
  tf2::Quaternion q;
  q.setRPY(0, 0, pos.z());
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending Nav2 goal to (%.2f, %.2f, %.2f)",
    name_.c_str(), pos.x(), pos.y(), pos.z());

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      } else {
        active_goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult& result) {
      if (active_execution_)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          RCLCPP_INFO(node_->get_logger(), "[%s] Goal reached successfully", name_.c_str());
          active_execution_->finished();
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "[%s] Goal failed or was cancelled", name_.c_str());
        }
        active_execution_.reset();
      }
      active_goal_handle_ = nullptr;
    };

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void Nav2RobotHandle::stop(rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr)
{
  if (active_goal_handle_)
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] Cancelling Nav2 goal", name_.c_str());
    nav_client_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_ = nullptr;
  }
  
  if (active_execution_)
  {
    active_execution_.reset();
  }
}

// FleetAdapter Side.