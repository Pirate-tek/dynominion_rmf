#ifndef DYNOMINION_FLEET_ADAPTER__NAV_CONTROLLER_HPP_
#define DYNOMINION_FLEET_ADAPTER__NAV_CONTROLLER_HPP_

/// @file NavController.hpp
/// @brief Goal 3 — Custom Navigation Controller.
///
/// Waypoint execution engine that replaces direct Nav2 goal forwarding with a
/// deliberate, rule-governed control loop. It receives navigation requests directly from
/// FleetManagerNode via Web API, processes them, enforces heading pre-rotation, approach
/// speed reduction, and three overlay rules (pause, obstacle, cancel).
///
/// Waypoint advancement is gated on a successful Nav2Handler result — distance
/// threshold is an early indicator only.

#include <dynominion_fleet_adapter/Nav2Handler.hpp>
#include <dynominion_fleet_adapter/RMFHandler.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <functional>
#include <optional>

namespace dynominion_fleet_adapter
{

/// @brief Goal 3 — Navigation Controller per robot.
///
/// Lifecycle per task:
///   HTTP request received

///     → pre-rotation to align heading (if error > threshold)
///     → translation with approach-zone speed reduction
///     → waypoint reached (distance + Nav2 success)
///     → repeat for next waypoint
///     → final yaw alignment on last waypoint
///     → CommandExecution::finished()
///
/// Overlay rules (checked every control tick, highest priority):
///   1. Cancel  — flush queue immediately, cancel Nav2, drop execution
///   2. Obstacle — halt translation+rotation, hold until clear or cancel
///   3. Pause   — suspend translation, allow active rotation to complete
class NavController
{
public:
  using EasyFleet      = rmf_fleet_adapter::agv::EasyFullControl;
  using CommandExec    = EasyFleet::CommandExecution;

  /// @param node             Shared ROS2 node.
  /// @param rmf_handler      RMFHandler — for StateGuard pose/battery updates.
  /// @param nav2_handler     Nav2Handler — forwards pose goals to Nav2.
  /// @param robot_name       Robot namespace (e.g. "dynominion1").
  /// @param fleet_name       Fleet name (e.g. "dynominion_fleet").
  NavController(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<RMFHandler> & rmf_handler,
    const std::shared_ptr<Nav2Handler> & nav2_handler,
    const std::string & robot_name,
    const std::string & fleet_name);

  /// Start a navigation task to a single coordinate.
  void navigate(double x, double y, double yaw, const std::string& task_id);

  /// Trigger a pause (suspend translation). Called from the RMF pause callback.

  void request_pause();

  /// Resume after a pause.
  void request_resume();

  /// Cancel all active navigation immediately (called from on_stop).
  void cancel_all();

  /// Update the live pose estimate from external sensors/odometry (e.g. AMCL).
  void update_pose(double x, double y, double yaw);

  // ── Goal 5 — State Machine integration callbacks ───────────────────────
 
  /// Register a callback invoked when a new task starts executing.
  using TaskStartedCallback  = std::function<void(const std::string & task_id)>;
  /// Register a callback invoked when a goal is accepted by Nav2.
  using TaskAcceptedCallback = std::function<void(const std::string & task_id)>;
  /// Register a callback invoked when a task terminates.
  /// @param success  true = arrived; false = aborted/cancelled.
  /// @param reason   empty on success; human-readable on failure.
  using TaskFinishedCallback = std::function<void(bool success, const std::string & reason)>;
 
  void set_on_task_started(TaskStartedCallback cb)   { on_task_started_   = std::move(cb); }
  void set_on_task_accepted(TaskAcceptedCallback cb) { on_task_accepted_  = std::move(cb); }
  void set_on_task_finished(TaskFinishedCallback cb) { on_task_finished_  = std::move(cb); }
 
private:
  void on_nav2_accepted(bool accepted);
  // ── Internal control state machine ─────────────────────────────────────

  enum class Phase
  {
    IDLE,            ///< No active task
    PRE_ROTATION,    ///< Rotating to align heading before translation
    TRANSLATING,     ///< Moving toward current waypoint
    APPROACH,        ///< Within approach radius — reduced speed
    FINAL_ROTATION,  ///< Final in-place rotation at last waypoint
    WAITING_NAV2,    ///< Distance reached; awaiting Nav2 result confirmation
    COMPLETE,        ///< Task complete — calling finished()
    CANCELLED,       ///< Cancellation in progress
  };

  // ── Loaded from nav parameters ──────────────────────────────────────────
  double heading_error_threshold_{0.35};   ///< rad
  double approach_radius_{0.5};            ///< m
  double approach_speed_limit_{0.2};       ///< m/s
  double arrival_threshold_{0.15};         ///< m

  // ── Control loop ───────────────────────────────────────────────────────

  /// 50 Hz control tick — evaluates overlay rules, drives state machine.
  void control_tick();

  /// Dispatch the next waypoint in the queue to Nav2.
  void dispatch_next_waypoint();

  /// Called by Nav2Handler when the current goal terminates.
  void on_nav2_result(Nav2Handler::GoalResult result);

  /// Called by Nav2Handler on each feedback message with a new pose estimate.
  void on_nav2_pose(double x, double y, double yaw);

  /// Compute heading error from current pose to target waypoint.
  double compute_heading_error(double tx, double ty) const;

  /// Compute Euclidean distance from current pose to (tx, ty).
  double compute_distance(double tx, double ty) const;

  /// Publish a Twist command on /<robot_name>/cmd_vel.
  void publish_cmd_vel(double linear_x, double angular_z);

  /// Complete the current task and notify RMF.
  void complete_task();

  /// Abort the current task without notifying finished() (used on cancel).
  void abort_task();

  // ── Members ─────────────────────────────────────────────────────────────

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<RMFHandler>  rmf_handler_;
  std::shared_ptr<Nav2Handler> nav2_handler_;
  std::string robot_name_;
  std::string fleet_name_;


  /// Subscription: /<robot_name>/obstacle_detected  (std_msgs/Bool)
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;

  /// Publisher: /<robot_name>/cmd_vel  (for pre-rotation commands)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  /// 50 Hz control loop timer.
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ── Task state — protected by task_mtx_ ─────────────────────────────────

  mutable std::mutex task_mtx_;

  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_yaw_ = 0.0;
  bool target_has_yaw_ = false;
  std::string current_task_id_;

  Phase phase_ = Phase::IDLE;


  // Live pose (updated from Nav2 feedback and StateGuard)
  double pose_x_   = 0.0;
  double pose_y_   = 0.0;
  double pose_yaw_ = 0.0;
  bool   pose_valid_ = false;

  // Overlay flags
  std::atomic<bool> paused_{false};
  std::atomic<bool> obstacle_detected_{false};
  std::atomic<bool> cancel_requested_{false};

  // Nav2 result latch (set in Nav2 callback, consumed in control tick)
  std::optional<Nav2Handler::GoalResult> nav2_result_;


  // ── Goal 5 hooks ───────────────────────────────────────────────────────
  std::optional<TaskStartedCallback>  on_task_started_;
  std::optional<TaskAcceptedCallback> on_task_accepted_;
  std::optional<TaskFinishedCallback> on_task_finished_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__NAV_CONTROLLER_HPP_
