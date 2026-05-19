#include <dynominion_fleet_adapter/RMFHandler.hpp>


#include <sstream>

namespace dynominion_fleet_adapter
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

RMFHandler::RMFHandler(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<FleetValidator> & validator,
  const std::string & robot_name)
: node_(node),
  validator_(validator),
  robot_name_(robot_name)
{
  RCLCPP_INFO(node_->get_logger(),
    "[RMFHandler][%s] Initialised.", robot_name_.c_str());
}

// ---------------------------------------------------------------------------
// EasyFullControl callback — navigate
// ---------------------------------------------------------------------------

std::optional<std::string> RMFHandler::on_navigate(Destination destination)
{
  // ── Generate a synthetic task ID ─────────────────────────────────────────
  std::string task_id;
  {
    std::lock_guard<std::mutex> lk(state_.mtx);
    task_id = robot_name_ + "_task_" + std::to_string(++task_counter_);
    state_.active_task_id = task_id;
  }

  const auto & p = destination.position();
  RCLCPP_INFO(node_->get_logger(),
    "[RMFHandler][%s] Received RMF navigate request: task_id='%s' goal=(%.2f, %.2f) waypoint='%s'",
    robot_name_.c_str(), task_id.c_str(), p.x(), p.y(), destination.name().c_str());

  // ── Validate task type ───────────────────────────────────────────────────
  const std::string task_type = "go_to_place";
  ValidationResult vr = validator_->validate_task(task_type, task_id, robot_name_);

  if (!vr.accepted)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[RMFHandler][%s] Rejecting navigation command. Reason: %s",
      robot_name_.c_str(), vr.reason.c_str());

    // Clear the active task ID on rejection
    {
      std::lock_guard<std::mutex> lk(state_.mtx);
      state_.active_task_id.clear();
    }

    return std::nullopt;
  }

  return task_id;
}

// ---------------------------------------------------------------------------
// EasyFullControl callback — stop
// ---------------------------------------------------------------------------

void RMFHandler::on_stop(ActivityId /*identifier*/)
{
  RCLCPP_INFO(node_->get_logger(),
    "[RMFHandler][%s] Stop requested by RMF. Clearing active task.", robot_name_.c_str());

  std::lock_guard<std::mutex> lk(state_.mtx);
  state_.active_task_id.clear();

  // Stop is passed via HTTP API now.
}

// ---------------------------------------------------------------------------
// StateGuard accessors
// ---------------------------------------------------------------------------

void RMFHandler::update_pose(double x, double y, double yaw)
{
  std::lock_guard<std::mutex> lk(state_.mtx);
  state_.current_pose = {x, y, yaw};
  state_.localized = true;
}

void RMFHandler::update_battery(double soc)
{
  std::lock_guard<std::mutex> lk(state_.mtx);
  state_.battery_soc = soc;
}

void RMFHandler::set_localized(bool localized)
{
  std::lock_guard<std::mutex> lk(state_.mtx);
  state_.localized = localized;
}

RMFHandler::StateSnapshot RMFHandler::get_state_snapshot() const
{
  std::lock_guard<std::mutex> lk(state_.mtx);
  return StateSnapshot{
    state_.current_pose,
    state_.active_task_id,
    state_.battery_soc,
    state_.localized
  };
}



}  // namespace dynominion_fleet_adapter
