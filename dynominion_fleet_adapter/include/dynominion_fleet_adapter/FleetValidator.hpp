#ifndef DYNOMINION_FLEET_ADAPTER__FLEET_VALIDATOR_HPP_
#define DYNOMINION_FLEET_ADAPTER__FLEET_VALIDATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <unordered_set>

namespace dynominion_fleet_adapter
{

/// @brief Stamped descriptor for a task that has passed validation.
///
/// Produced by FleetValidator::validate_task() on success and forwarded
/// downstream to the RMFHandler / topic bridge.
struct ValidatedTaskDescriptor
{
  std::string task_id;     ///< Unique task identifier from RMF
  std::string task_type;   ///< Validated task type string (e.g. "go_to_place")
  std::string robot_name;  ///< Target robot name
};

/// @brief Result type returned by FleetValidator::validate_task().
struct ValidationResult
{
  bool accepted;           ///< true if the task passed all checks
  std::string reason;      ///< Human-readable rejection reason (empty on accept)
  ValidatedTaskDescriptor descriptor;  ///< Populated only when accepted == true
};

/// @brief Goal-1 — Fleet Identity & Task Validation Layer.
///
/// Reads fleet identity parameters from the ROS2 node on construction and
/// validates them against an expected schema (presence, type, non-empty).
/// If startup validation fails, is_valid() returns false and the caller
/// MUST refuse to enter the RMF schedule.
///
/// At runtime, validate_task() checks incoming task types against the
/// declared supported_tasks list and returns a ValidationResult.
class FleetValidator
{
public:
  /// Construct and immediately validate startup parameters.
  /// @param node  The ROS2 node — parameters must already be declared
  ///              (or loaded via --params-file) before calling this ctor.
  explicit FleetValidator(const rclcpp::Node::SharedPtr & node);

  /// @returns true if all startup parameters were valid.
  bool is_valid() const { return valid_; }

  /// @returns fleet_name declared in the parameter YAML.
  const std::string & fleet_name() const { return fleet_name_; }

  /// @returns fleet_type declared in the parameter YAML.
  const std::string & fleet_type() const { return fleet_type_; }

  /// @returns supported task types declared in the parameter YAML.
  const std::vector<std::string> & supported_tasks() const { return supported_tasks_; }

  /// Validate an incoming task against the declared contract.
  /// @param task_type   Task type string delivered by RMF (e.g. "go_to_place").
  /// @param task_id     Unique task ID from RMF (for logging / descriptor).
  /// @param robot_name  Target robot name (for descriptor).
  /// @returns ValidationResult — check .accepted before using .descriptor.
  ValidationResult validate_task(
    const std::string & task_type,
    const std::string & task_id,
    const std::string & robot_name) const;

private:
  /// Declare and read all fleet identity parameters; sets valid_ accordingly.
  void load_and_validate_parameters();

  rclcpp::Node::SharedPtr node_;
  bool valid_ = false;

  std::string fleet_name_;
  std::string fleet_type_;
  std::vector<std::string> supported_tasks_;
  std::unordered_set<std::string> supported_tasks_set_;  ///< O(1) lookup
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__FLEET_VALIDATOR_HPP_
