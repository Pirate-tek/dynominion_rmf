#ifndef DYNOMINION_FLEET_ADAPTER__RMF_HANDLER_HPP_
#define DYNOMINION_FLEET_ADAPTER__RMF_HANDLER_HPP_

#include <dynominion_fleet_adapter/FleetValidator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>


#include <mutex>
#include <string>
#include <memory>
#include <optional>

namespace dynominion_fleet_adapter
{

// ---------------------------------------------------------------------------
// StateGuard — thread-safe shared robot state
// ---------------------------------------------------------------------------

/// @brief Goal-2 — StateGuard protects all shared robot state.
///
/// The RMF callback fires on RMF's internal thread. The ROS2 executor runs
/// on a separate thread. Every read and write to fields below MUST be done
/// while holding the mutex.
struct StateGuard
{
  mutable std::mutex mtx;

  /// Latest robot pose in map frame [x, y, yaw].
  std::optional<std::array<double, 3>> current_pose;

  /// RMF task ID currently assigned to this robot, empty if idle.
  std::string active_task_id;

  /// Battery state-of-charge in [0.0, 1.0].
  double battery_soc = 1.0;

  /// True once the robot has sent at least one valid pose to RMF.
  bool localized = false;
};

// ---------------------------------------------------------------------------
// RMFHandler
// ---------------------------------------------------------------------------

/// @brief Goal-2 — RMF Handler & Topic Bridge.
///
/// Owns the EasyFullControl callback surface and acts as the sole point of
/// contact between the in-process RMF scheduler and the rest of the adapter.
///
/// When FleetValidator rejects a task, RMFHandler returns nullopt and 
/// the caller should reject the execution.
class RMFHandler
{
public:
  using EasyFleet   = rmf_fleet_adapter::agv::EasyFullControl;
  using Destination = EasyFleet::Destination;
  using CommandExec = EasyFleet::CommandExecution;
  using ActivityId  = EasyFleet::ConstActivityIdentifierPtr;

  /// @param node        Shared ROS2 node — used for pub/sub and logging.
  /// @param validator   Initialised FleetValidator — must be valid().
  /// @param robot_name  Name of the robot this handler is responsible for.
  RMFHandler(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<FleetValidator> & validator,
    const std::string & robot_name);

  // ── EasyFullControl callback surface ─────────────────────────────────────

  /// Navigate callback registered with EasyFullControl.
  /// Validates the task and generates a task ID. Returns task_id if accepted, nullopt if rejected.
  std::optional<std::string> on_navigate(Destination destination);

  /// Stop callback registered with EasyFullControl.
  void on_stop(ActivityId identifier);

  // ── StateGuard accessors ─────────────────────────────────────────────────

  /// Update the cached robot pose (called from the ROS2 executor thread).
  void update_pose(double x, double y, double yaw);

  /// Update the battery estimate (called from the ROS2 executor thread).
  void update_battery(double soc);

  /// Mark the robot as localized.
  void set_localized(bool localized);

  /// Read a snapshot of the current robot state (thread-safe).
  struct StateSnapshot
  {
    std::optional<std::array<double, 3>> pose;
    std::string active_task_id;
    double battery_soc;
    bool localized;
  };
  StateSnapshot get_state_snapshot() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<FleetValidator> validator_;
  std::string robot_name_;


  /// Thread-safe shared state.
  StateGuard state_;

  /// Running counter for generating synthetic task IDs when RMF does not
  /// supply one (navigation-only calls typically do not carry a task ID).
  uint64_t task_counter_ = 0;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__RMF_HANDLER_HPP_
