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
  std::lock_guard<std::mutex> lock(mutex_);

  if (active_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[Nav2Handler][%s] send_goal called while a goal is already active. Queueing pending goal.",
      robot_name_.c_str());

    if (active_goal_handle_)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[Nav2Handler][%s] Cancelling active Nav2 goal to clear way for pending goal.",
        robot_name_.c_str());
      action_client_->async_cancel_goal(active_goal_handle_);
    }

    pending_goal_ = std::make_unique<PendingGoal>(PendingGoal{
      x, y, yaw,
      std::move(on_result),
      std::move(on_pose),
      std::move(on_accepted)
    });
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
      AcceptedCallback local_on_accepted = on_accepted;
      ResultCallback local_result_cb;
      std::unique_ptr<PendingGoal> next_goal;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!handle)
        {
          RCLCPP_ERROR(node_->get_logger(),
            "[Nav2Handler][%s] Goal was REJECTED by Nav2 server.", robot_name_.c_str());
          active_ = false;
          local_result_cb = std::move(pending_result_cb_);

          if (pending_goal_)
          {
            next_goal = std::move(pending_goal_);
          }
        }
        else
        {
          active_goal_handle_ = handle;
          RCLCPP_DEBUG(node_->get_logger(),
            "[Nav2Handler][%s] Goal accepted by Nav2.", robot_name_.c_str());
        }
      }

      if (!handle)
      {
        if (local_on_accepted)
          local_on_accepted(false);
        if (local_result_cb)
          local_result_cb(GoalResult::ABORTED);

        if (next_goal)
        {
          send_goal(next_goal->x, next_goal->y, next_goal->yaw,
                    std::move(next_goal->on_result),
                    std::move(next_goal->on_pose),
                    std::move(next_goal->on_accepted));
        }
      }
      else
      {
        if (local_on_accepted)
          local_on_accepted(true);
      }
    };

  send_options.feedback_callback =
    [this](GoalHandle::SharedPtr /*handle*/,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      PoseCallback local_pose_cb;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        local_pose_cb = pending_pose_cb_;
      }

      if (local_pose_cb)
      {
        const auto & p = feedback->current_pose.pose.position;
        const auto & o = feedback->current_pose.pose.orientation;

        // Convert quaternion to yaw
        tf2::Quaternion q_fb;
        tf2::fromMsg(o, q_fb);
        double roll, pitch, yaw_fb;
        tf2::Matrix3x3(q_fb).getRPY(roll, pitch, yaw_fb);

        local_pose_cb(p.x, p.y, yaw_fb);
      }
    };

  send_options.result_callback =
    [this](const GoalHandle::WrappedResult & result)
    {
      ResultCallback local_result_cb;
      std::unique_ptr<PendingGoal> next_goal;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_            = false;
        active_goal_handle_ = nullptr;
        local_result_cb    = std::move(pending_result_cb_);

        if (pending_goal_)
        {
          next_goal = std::move(pending_goal_);
        }
      }

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

      if (local_result_cb)
        local_result_cb(gr);

      if (next_goal)
      {
        RCLCPP_INFO(node_->get_logger(),
          "[Nav2Handler][%s] Sending pending goal from queue.", robot_name_.c_str());
        send_goal(next_goal->x, next_goal->y, next_goal->yaw,
                  std::move(next_goal->on_result),
                  std::move(next_goal->on_pose),
                  std::move(next_goal->on_accepted));
      }
    };

  action_client_->async_send_goal(goal_msg, send_options);
}

// ---------------------------------------------------------------------------
// cancel_goal
// ---------------------------------------------------------------------------

void Nav2Handler::cancel_goal()
{
  std::lock_guard<std::mutex> lock(mutex_);

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
    });
}

// ---------------------------------------------------------------------------
// clear_pending_goal
// ---------------------------------------------------------------------------

void Nav2Handler::clear_pending_goal()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_goal_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[Nav2Handler][%s] Clearing pending goal from queue.", robot_name_.c_str());
    pending_goal_.reset();
  }
}

}  // namespace dynominion_fleet_adapter
