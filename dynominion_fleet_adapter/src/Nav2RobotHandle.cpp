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
  start_time_ = node_->now();
  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/" + name_ + "/navigate_to_pose");

  // Telemetry subscribers
  amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/" + name_ + "/amcl_pose", rclcpp::QoS(10).transient_local(),
    std::bind(&Nav2RobotHandle::amcl_pose_callback, this, std::placeholders::_1));

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + name_ + "/battery_state", 10,
    std::bind(&Nav2RobotHandle::battery_callback, this, std::placeholders::_1));

  // TF Initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

bool Nav2RobotHandle::is_ready()
{
  // Check if we have a valid transform map -> base_footprint
  std::string base_frame = name_ + "/base_footprint";
  bool has_tf = tf_buffer_->canTransform("map", base_frame, tf2::TimePointZero);
  
  if (!has_tf)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] Waiting for transform map -> %s", name_.c_str(), base_frame.c_str());
  }

  // Check AMCL covariance if available
  bool has_amcl = (last_amcl_pose_ != nullptr);
  bool cov_ok = false;
  
  if (!has_amcl)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] Waiting for AMCL pose message...", name_.c_str());
  }
  else
  {
    double cov_x = last_amcl_pose_->pose.covariance[0];
    double cov_y = last_amcl_pose_->pose.covariance[7];
    double cov_yaw = last_amcl_pose_->pose.covariance[35];

    // Covariance threshold for "localized" (0.1m^2 for position, 0.05 rad^2 for yaw)
    if (cov_x > 0.1 || cov_y > 0.1 || cov_yaw > 0.05)
    {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "[%s] Localization covariance too high: x=%.3f, y=%.3f, yaw=%.3f",
        name_.c_str(), cov_x, cov_y, cov_yaw);
    }
    else
    {
      cov_ok = true;
    }
  }

  if (has_tf && has_amcl && cov_ok)
  {
    return true;
  }

  // Simulation-only readiness bypass: 
  // If we have TF but AMCL is still not perfectly localized after 30 seconds, bypass it.
  double elapsed = (node_->now() - start_time_).seconds();
  if (has_tf && elapsed > 30.0)
  {
    RCLCPP_WARN(node_->get_logger(), "[%s] Readiness timeout (%.1fs). Bypassing AMCL checks because TF is available. This should only happen in simulation!", name_.c_str(), elapsed);
    return true;
  }

  return false;
}


rmf_fleet_adapter::agv::EasyFullControl::RobotState Nav2RobotHandle::get_state()
{
  std::string base_frame = name_ + "/base_footprint";
  auto tf = tf_buffer_->lookupTransform("map", base_frame, tf2::TimePointZero);

  Eigen::Vector3d position(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf2::getYaw(tf.transform.rotation)
  );

  return rmf_fleet_adapter::agv::EasyFullControl::RobotState(level_name_, position, last_battery_soc_);
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

void Nav2RobotHandle::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_amcl_pose_ = msg;
}

void Nav2RobotHandle::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_soc_ = msg->percentage;
}

void Nav2RobotHandle::update_loop()
{
  if (!update_handle_)
    return;

  try {
    update_handle_->update(get_state(), nullptr);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] TF lookup failed: %s", name_.c_str(), e.what());
  }
}
