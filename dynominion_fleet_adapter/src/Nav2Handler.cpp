#include <dynominion_fleet_adapter/Nav2Handler.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dynominion_fleet_adapter
{

Nav2Handler::Nav2Handler(
  const rclcpp::Node::SharedPtr & node,
  const std::string & robot_name)
: node_(node),
  robot_name_(robot_name)
{
  const std::string action_name = "/" + robot_name_ + "/navigate_to_pose";
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);

  RCLCPP_INFO(node_->get_logger(),
    "[Nav2Handler][%s] Initialised action client on '%s'.",
    robot_name_.c_str(), action_name.c_str());
}

// ---------------------------------------------------------------------------
// send_goal
// ---------------------------------------------------------------------------

void Nav2Handler::send_goal(
  double x, double y, double yaw,
  ResultCallback on_result,
  PoseCallback on_pose,
  AcceptedCallback on_accepted)
{
  if (active_)
  {
    RCLCPP_WARN(node_->get_logger(),
      "[Nav2Handler][%s] send_goal called while a goal is already active. Ignoring.",
      robot_name_.c_str());
    if (on_accepted)
      on_accepted(false);
    return;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(0)))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[Nav2Handler][%s] Nav2 action server not available. Goal dropped.",
      robot_name_.c_str());
    if (on_accepted)
      on_accepted(false);
    if (on_result)
      on_result(GoalResult::ABORTED);
    return;
  }

  pending_result_cb_ = std::move(on_result);
  pending_pose_cb_   = std::move(on_pose);
  active_            = true;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp    = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node_->get_logger(),
    "[Nav2Handler][%s] Sending goal -> (%.2f, %.2f, yaw=%.2f).",
    robot_name_.c_str(), x, y, yaw);

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_options.goal_response_callback =
    [this, on_accepted](const GoalHandle::SharedPtr & handle)
    {
      if (!handle)
      {
        RCLCPP_ERROR(node_->get_logger(),
          "[Nav2Handler][%s] Goal was REJECTED by Nav2 server.", robot_name_.c_str());
        active_ = false;
        if (on_accepted)
          on_accepted(false);
        if (pending_result_cb_)
          pending_result_cb_(GoalResult::ABORTED);
      }
      else
      {
        active_goal_handle_ = handle;
        RCLCPP_DEBUG(node_->get_logger(),
          "[Nav2Handler][%s] Goal accepted by Nav2.", robot_name_.c_str());
        if (on_accepted)
          on_accepted(true);
      }
    };

  send_options.feedback_callback =
    [this](GoalHandle::SharedPtr /*handle*/,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      // Extract current pose from feedback and forward to NavController/StateGuard
      const auto & p = feedback->current_pose.pose.position;
      const auto & o = feedback->current_pose.pose.orientation;

      // Convert quaternion to yaw
      tf2::Quaternion q_fb;
      tf2::fromMsg(o, q_fb);
      double roll, pitch, yaw_fb;
      tf2::Matrix3x3(q_fb).getRPY(roll, pitch, yaw_fb);

      if (pending_pose_cb_)
        pending_pose_cb_(p.x, p.y, yaw_fb);
    };

  send_options.result_callback =
    [this](const GoalHandle::WrappedResult & result)
    {
      active_            = false;
      active_goal_handle_ = nullptr;

      GoalResult gr;
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(),
            "[Nav2Handler][%s] Goal SUCCEEDED.", robot_name_.c_str());
          gr = GoalResult::SUCCEEDED;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(node_->get_logger(),
            "[Nav2Handler][%s] Goal CANCELLED.", robot_name_.c_str());
          gr = GoalResult::CANCELLED;
          break;
        default:
          RCLCPP_WARN(node_->get_logger(),
            "[Nav2Handler][%s] Goal ABORTED.", robot_name_.c_str());
          gr = GoalResult::ABORTED;
          break;
      }

      if (pending_result_cb_)
        pending_result_cb_(gr);
    };

  action_client_->async_send_goal(goal_msg, send_options);
}

// ---------------------------------------------------------------------------
// cancel_goal
// ---------------------------------------------------------------------------

void Nav2Handler::cancel_goal()
{
  if (!active_ || !active_goal_handle_)
  {
    RCLCPP_DEBUG(node_->get_logger(),
      "[Nav2Handler][%s] cancel_goal called but no active goal.", robot_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[Nav2Handler][%s] Cancelling active Nav2 goal (async).", robot_name_.c_str());

  action_client_->async_cancel_goal(
    active_goal_handle_,
    [this](const auto & /*cancel_response*/)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[Nav2Handler][%s] Async cancel accepted by Nav2.", robot_name_.c_str());
      // The result_callback will fire next with CANCELED, which triggers
      // the pending_result_cb_ and notifies the NavController sequencer.
    });
}

}  // namespace dynominion_fleet_adapter
