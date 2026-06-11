#include <dynominion_fleet_adapter/RobotStateMachine.hpp>

namespace dynominion_fleet_adapter
{

RobotStateMachine::RobotStateMachine(
  const std::string & robot_name,
  const rclcpp::Logger & logger)
: robot_name_(robot_name),
  logger_(logger)
{
  RCLCPP_INFO(logger_,
    "[StateMachine][%s] Initialised in state IDLE.", robot_name_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Transition triggers
// ─────────────────────────────────────────────────────────────────────────────

bool RobotStateMachine::on_navigate(const std::string & task_id)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (state_ != RobotState::IDLE && state_ != RobotState::NAVIGATING)
  {
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] navigate() rejected — already in state %s (task='%s').",
      robot_name_.c_str(), to_string(state_), active_task_id_.c_str());
    return false;
  }

  if (state_ == RobotState::NAVIGATING)
  {
    RCLCPP_INFO(logger_,
      "[StateMachine][%s] navigate() preempting task '%s' for task '%s'. Resetting to IDLE state first.",
      robot_name_.c_str(), active_task_id_.c_str(), task_id.c_str());
    state_ = RobotState::IDLE;
  }

  active_task_id_    = task_id;
  last_error_reason_ = {};
  RCLCPP_INFO(logger_,
    "[StateMachine][%s] navigate() request initiated for task '%s'. Awaiting Nav2 acceptance...",
    robot_name_.c_str(), task_id.c_str());
  return true;
}

bool RobotStateMachine::on_navigate_accepted()
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (state_ != RobotState::IDLE)
  {
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] navigate_accepted() rejected — state is %s, expected IDLE.",
      robot_name_.c_str(), to_string(state_));
    return false;
  }

  if (active_task_id_.empty())
  {
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] navigate_accepted() called but active_task_id is empty.",
      robot_name_.c_str());
    return false;
  }

  trace_task(active_task_id_, to_string(RobotState::NAVIGATING));
  return transition(RobotState::NAVIGATING, "task=" + active_task_id_);
}

bool RobotStateMachine::on_success()
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (state_ != RobotState::NAVIGATING)
  {
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] success() rejected — not in NAVIGATING (state=%s).",
      robot_name_.c_str(), to_string(state_));
    return false;
  }

  const std::string tid = active_task_id_;
  const std::string ctx = "task=" + active_task_id_ + " completed";
  active_task_id_.clear();
  trace_task(tid, to_string(RobotState::IDLE));
  return transition(RobotState::IDLE, ctx);
}

bool RobotStateMachine::on_error(const std::string & reason)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (state_ == RobotState::ERROR)
  {
    // Already in error — just update the reason
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] error() called while already in ERROR (reason=%s).",
      robot_name_.c_str(), reason.c_str());
    last_error_reason_ = reason;
    return true;
  }

  if (state_ == RobotState::IDLE)
  {
    if (!active_task_id_.empty())
    {
      RCLCPP_WARN(logger_,
        "[StateMachine][%s] Goal rejected at startup for task '%s'. Transitioning to ERROR.",
        robot_name_.c_str(), active_task_id_.c_str());
      last_error_reason_ = reason;
      const std::string tid = active_task_id_;
      const std::string ctx = "task=" + active_task_id_ + " failed at startup: " + reason;
      active_task_id_.clear();
      trace_task(tid, to_string(RobotState::ERROR));
      return transition(RobotState::ERROR, ctx);
    }

    RCLCPP_WARN(logger_,
      "[StateMachine][%s] error() called from IDLE with no active task — treating as spurious. "
      "Reason: %s",
      robot_name_.c_str(), reason.c_str());
    return false;
  }

  last_error_reason_ = reason;
  const std::string tid = active_task_id_;
  const std::string ctx = "task=" + active_task_id_ + " failed: " + reason;
  active_task_id_.clear();
  trace_task(tid, to_string(RobotState::ERROR));
  return transition(RobotState::ERROR, ctx);
}

bool RobotStateMachine::on_recover()
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (state_ != RobotState::ERROR)
  {
    RCLCPP_WARN(logger_,
      "[StateMachine][%s] recover() rejected — not in ERROR (state=%s).",
      robot_name_.c_str(), to_string(state_));
    return false;
  }

  last_error_reason_.clear();
  trace_task("recovery", to_string(RobotState::IDLE));
  return transition(RobotState::IDLE, "manual recovery");
}

// ─────────────────────────────────────────────────────────────────────────────
// Observers
// ─────────────────────────────────────────────────────────────────────────────

RobotState RobotStateMachine::state() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return state_;
}

std::string RobotStateMachine::active_task_id() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return active_task_id_;
}

std::string RobotStateMachine::last_error_reason() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return last_error_reason_;
}

void RobotStateMachine::set_transition_callback(TransitionCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  transition_cb_ = std::move(cb);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal
// ─────────────────────────────────────────────────────────────────────────────

bool RobotStateMachine::transition(RobotState to, const std::string & context)
{
  // Must be called with mtx_ held.
  RobotState from = state_;
  state_ = to;

  RCLCPP_INFO(logger_,
    "[StateMachine][%s] %s → %s  (%s)",
    robot_name_.c_str(),
    to_string(from), to_string(to),
    context.c_str());

  if (transition_cb_.has_value())
  {
    // Fire callback without the lock to avoid re-entrance deadlocks.
    // Copy the callback so the lock can be released.
    auto cb = *transition_cb_;
    // We unlock-invoke-relock pattern is not needed here since the callback
    // is called after all state mutations are done and before the function
    // returns. The mutex is still held, so the callback must not re-enter.
    cb(from, to, context);
  }

  return true;
}

void RobotStateMachine::trace_task(
  const std::string& task_id,
  const std::string& state) const
{
  RCLCPP_WARN(
    logger_,
    "[TASK TRACE] %s -> %s",
    task_id.c_str(),
    state.c_str());
}

}  // namespace dynominion_fleet_adapter
