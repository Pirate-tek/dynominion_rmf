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

  // Telemetry subscribers
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/" + name_ + "/odom", 10,
    std::bind(&Nav2RobotHandle::odom_callback, this, std::placeholders::_1));

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + name_ + "/battery_state", 10,
    std::bind(&Nav2RobotHandle::battery_callback, this, std::placeholders::_1));

  // RMF update loop (500ms)
  update_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&Nav2RobotHandle::update_loop, this));

  RCLCPP_INFO(node_->get_logger(), "Initialized Nav2RobotHandle for %s", name_.c_str());
}

void Nav2RobotHandle::set_update_handle(const std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle>& update_handle)
{
  update_handle_ = update_handle;
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

void Nav2RobotHandle::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_ = msg;
}

void Nav2RobotHandle::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_soc_ = msg->percentage;
}

void Nav2RobotHandle::update_loop()
{
  if (!last_odom_ || !update_handle_)
    return;

  Eigen::Vector3d position(
    last_odom_->pose.pose.position.x,
    last_odom_->pose.pose.position.y,
    tf2::getYaw(last_odom_->pose.pose.orientation)
  );

  update_handle_->update(
    rmf_fleet_adapter::agv::EasyFullControl::RobotState("L1", position, last_battery_soc_),
    nullptr
  );
}
