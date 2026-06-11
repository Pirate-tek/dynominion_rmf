#ifndef DYNOMINION_FLEET_ADAPTER__NAV2_HANDLER_HPP_
#define DYNOMINION_FLEET_ADAPTER__NAV2_HANDLER_HPP_

/// @file Nav2Handler.hpp
/// @brief Goal 4 — Nav2Handler: one NavigateToPose action client per robot,
///        isolating the rest of the adapter from Nav2's action protocol details.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <dynominion_fleet_adapter/RMFHandler.hpp>   // for StateGuard access

#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <atomic>

namespace dynominion_fleet_adapter
{

/// @brief Goal 4 — Nav2Handler.
///
/// Manages all communication with the Nav2 stack for one robot, keyed by its
/// namespace. Accepts Pose goals from the NavController, sends them as
/// NavigateToPose action goals, extracts pose feedback into the StateGuard,
/// and reports goal results back via a result callback.
///
/// Cancellation is fully asynchronous — cancel_goal() never blocks the executor.
class Nav2Handler
{
public:
  using NavigateToPose  = nav2_msgs::action::NavigateToPose;
  using GoalHandle      = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /// Result code reported to the NavController sequencer.
  enum class GoalResult { SUCCEEDED, CANCELLED, ABORTED };

  /// Callback invoked on the ROS2 executor thread when a goal finishes.
  using ResultCallback = std::function<void(GoalResult)>;

  /// Callback invoked whenever Nav2 feedback carries a new pose estimate.
  /// Signature: (x, y, yaw)
  using PoseCallback = std::function<void(double, double, double)>;

  /// Callback invoked when a goal is accepted or rejected by the action server.
  using AcceptedCallback = std::function<void(bool)>;

  /// @param node         Shared ROS2 node.
  /// @param robot_name   Robot namespace (e.g. "dynominion1") — used to build
  ///                     the action server name "/<robot_name>/navigate_to_pose".
  Nav2Handler(
    const rclcpp::Node::SharedPtr & node,
    const std::string & robot_name);

  /// Send a NavigateToPose goal.
  /// @param x,y,yaw     Target pose in the "map" frame.
  /// @param on_result   Called when the goal terminates (any result).
  /// @param on_pose     Called on every feedback message with the current pose.
  /// @param on_accepted Called when the goal is accepted (true) or rejected/fails (false).
  void send_goal(
    double x, double y, double yaw,
    ResultCallback on_result,
    PoseCallback on_pose,
    AcceptedCallback on_accepted = nullptr);

  /// Asynchronously cancel the active goal.
  /// The on_result callback supplied to send_goal() will be invoked with
  /// GoalResult::CANCELLED once the cancellation is confirmed.
  void cancel_goal();

  /// @returns true if an active goal is in-flight.
  bool is_active() const { return active_; }

  /// Clear any queued pending goal.
  void clear_pending_goal();

private:
  struct PendingGoal
  {
    double x;
    double y;
    double yaw;
    ResultCallback on_result;
    PoseCallback on_pose;
    AcceptedCallback on_accepted;
  };

  rclcpp::Node::SharedPtr node_;
  std::string robot_name_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  GoalHandle::SharedPtr active_goal_handle_;
  std::atomic<bool> active_{false};

  ResultCallback pending_result_cb_;
  PoseCallback   pending_pose_cb_;

  std::unique_ptr<PendingGoal> pending_goal_;
  mutable std::mutex mutex_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__NAV2_HANDLER_HPP_
