#ifndef DYNOMINION_FLEET_ADAPTER__ROBOT_STATE_MACHINE_HPP_
#define DYNOMINION_FLEET_ADAPTER__ROBOT_STATE_MACHINE_HPP_

/// @file RobotStateMachine.hpp
/// @brief Goal 5 — State Machine
///
/// Governs the robot lifecycle state transitions:
///
///   IDLE ──navigate()──► NAVIGATING ──success()──► IDLE
///                             │
///                          error() / cancel()
///                             │
///                             ▼
///                           ERROR ──recover()──► IDLE
///
/// All transitions are logged and validated. Invalid transitions
/// (e.g. navigate() while NAVIGATING) are rejected with a warning.

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <functional>
#include <mutex>
#include <optional>

namespace dynominion_fleet_adapter
{

// ─────────────────────────────────────────────────────────────────────────────
// Robot lifecycle states
// ─────────────────────────────────────────────────────────────────────────────

enum class RobotState
{
  IDLE,        ///< Robot has no active task; ready to accept navigation goals.
  NAVIGATING,  ///< Robot is executing a navigation goal.
  ERROR,       ///< Robot encountered a fault; requires recovery before next goal.
};

/// @returns Human-readable name of a RobotState.
inline const char * to_string(RobotState s)
{
  switch (s)
  {
    case RobotState::IDLE:       return "IDLE";
    case RobotState::NAVIGATING: return "NAVIGATING";
    case RobotState::ERROR:      return "ERROR";
  }
  return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────────────
// RobotStateMachine
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Goal 5 — Robot lifecycle state machine.
///
/// Thread-safe state machine for a single robot. All public methods are
/// guarded by an internal mutex.  Optionally accepts a transition callback
/// that fires whenever state changes (useful for diagnostics/health monitors).
class RobotStateMachine
{
public:
  /// @param robot_name  Used for log messages.
  /// @param logger      ROS2 logger for state transition messages.
  explicit RobotStateMachine(
    const std::string & robot_name,
    const rclcpp::Logger & logger);

  // ── Transition triggers ──────────────────────────────────────────────────

  /// Begin navigating.  Only valid from IDLE.
  /// @param task_id  Used in log messages.
  /// @returns true if the transition was accepted.
  bool on_navigate(const std::string & task_id);

  /// Confirm navigation has been accepted by Nav2. Only valid from IDLE.
  /// Transitions state to NAVIGATING.
  /// @returns true if the transition was accepted.
  bool on_navigate_accepted();

  /// Navigation completed successfully.  Only valid from NAVIGATING.
  /// @returns true if the transition was accepted.
  bool on_success();

  /// Navigation failed or was cancelled.  Valid from NAVIGATING.
  /// Moves to ERROR so the fault is visible before recovery.
  /// @param reason  Human-readable failure description.
  /// @returns true if the transition was accepted.
  bool on_error(const std::string & reason);

  /// Clear the error and return to IDLE.  Only valid from ERROR.
  /// @returns true if the transition was accepted.
  bool on_recover();

  // ── Observers ─────────────────────────────────────────────────────────────

  RobotState state() const;
  bool is_idle()       const { return state() == RobotState::IDLE; }
  bool is_navigating() const { return state() == RobotState::NAVIGATING; }
  bool is_error()      const { return state() == RobotState::ERROR; }

  /// @returns the task_id of the active navigation goal, or empty string.
  std::string active_task_id() const;

  /// @returns the last error reason, or empty string if not in ERROR state.
  std::string last_error_reason() const;

  // ── Optional diagnostics callback ─────────────────────────────────────────

  using TransitionCallback =
    std::function<void(RobotState /*from*/, RobotState /*to*/, const std::string & /*context*/)>;

  /// Register a callback invoked on every state transition.
  void set_transition_callback(TransitionCallback cb);

private:
  /// Internal — perform a transition with logging. Must be called under mtx_.
  bool transition(RobotState to, const std::string & context);

  mutable std::mutex mtx_;

  void trace_task(
    const std::string& task_id,
    const std::string& state) const;

  const std::string robot_name_;
  rclcpp::Logger    logger_;

  RobotState  state_        {RobotState::IDLE};
  std::string active_task_id_;
  std::string last_error_reason_;

  std::optional<TransitionCallback> transition_cb_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__ROBOT_STATE_MACHINE_HPP_
