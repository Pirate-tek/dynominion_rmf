#include <dynominion_fleet_adapter/FleetValidator.hpp>
#include <sstream>

namespace dynominion_fleet_adapter
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

FleetValidator::FleetValidator(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  load_and_validate_parameters();
}

// ---------------------------------------------------------------------------
// Private — parameter loading
// ---------------------------------------------------------------------------

void FleetValidator::load_and_validate_parameters()
{
  const auto & logger = node_->get_logger();
  bool ok = true;

  // ── fleet_name ──────────────────────────────────────────────────────────
  node_->declare_parameter("fleet_identity.fleet_name", "");
  fleet_name_ = node_->get_parameter("fleet_identity.fleet_name").as_string();

  if (fleet_name_.empty())
  {
    RCLCPP_FATAL(logger,
      "[FleetValidator] Parameter 'fleet_identity.fleet_name' is missing or empty. "
      "The adapter cannot enter the RMF schedule without a fleet identity.");
    ok = false;
  }
  else
  {
    RCLCPP_INFO(logger, "[FleetValidator] fleet_name  = '%s'", fleet_name_.c_str());
  }

  // ── fleet_type ──────────────────────────────────────────────────────────
  node_->declare_parameter("fleet_identity.fleet_type", "");
  fleet_type_ = node_->get_parameter("fleet_identity.fleet_type").as_string();

  if (fleet_type_.empty())
  {
    RCLCPP_FATAL(logger,
      "[FleetValidator] Parameter 'fleet_identity.fleet_type' is missing or empty. "
      "Supported types: differential, omnidirectional, holonomic.");
    ok = false;
  }
  else
  {
    RCLCPP_INFO(logger, "[FleetValidator] fleet_type  = '%s'", fleet_type_.c_str());
  }

  // ── supported_tasks ─────────────────────────────────────────────────────
  node_->declare_parameter("fleet_identity.supported_tasks", std::vector<std::string>{});
  supported_tasks_ =
    node_->get_parameter("fleet_identity.supported_tasks").as_string_array();

  if (supported_tasks_.empty())
  {
    RCLCPP_FATAL(logger,
      "[FleetValidator] Parameter 'fleet_identity.supported_tasks' is missing or empty. "
      "At least one task type must be declared (e.g. [go_to_place, patrol]).");
    ok = false;
  }
  else
  {
    std::ostringstream oss;
    for (const auto & t : supported_tasks_)
    {
      supported_tasks_set_.insert(t);
      oss << t << " ";
    }
    RCLCPP_INFO(logger, "[FleetValidator] supported_tasks = [ %s]", oss.str().c_str());
  }

  valid_ = ok;

  if (valid_)
  {
    RCLCPP_INFO(logger,
      "[FleetValidator] Fleet identity validated successfully. "
      "Fleet '%s' (%s) is cleared to enter the RMF schedule.",
      fleet_name_.c_str(), fleet_type_.c_str());
  }
  else
  {
    RCLCPP_FATAL(logger,
      "[FleetValidator] Fleet identity validation FAILED. "
      "The adapter will refuse to register with the RMF schedule. "
      "Fix the parameter YAML and restart the node.");
  }
}

// ---------------------------------------------------------------------------
// validate_task
// ---------------------------------------------------------------------------

ValidationResult FleetValidator::validate_task(
  const std::string & task_type,
  const std::string & task_id,
  const std::string & robot_name) const
{
  ValidationResult result;

  if (supported_tasks_set_.count(task_type) == 0)
  {
    // Build a human-readable reason listing what IS supported
    std::ostringstream reason;
    reason << "Task type '" << task_type << "' is not in the supported_tasks contract for fleet '"
           << fleet_name_ << "'. Supported types are: [";
    for (const auto & t : supported_tasks_)
    {
      reason << " " << t;
    }
    reason << " ].";

    RCLCPP_WARN(node_->get_logger(),
      "[FleetValidator] REJECTED task '%s' (type='%s') for robot '%s'. %s",
      task_id.c_str(), task_type.c_str(), robot_name.c_str(), reason.str().c_str());

    result.accepted = false;
    result.reason = reason.str();
    return result;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[FleetValidator] ACCEPTED task '%s' (type='%s') for robot '%s'.",
    task_id.c_str(), task_type.c_str(), robot_name.c_str());

  result.accepted = true;
  result.reason.clear();
  result.descriptor = ValidatedTaskDescriptor{task_id, task_type, robot_name};
  return result;
}

}  // namespace dynominion_fleet_adapter
