#ifndef DYNOMINION_FLEET_ADAPTER__ABSTRACT_ROBOT_CLIENT_HPP_
#define DYNOMINION_FLEET_ADAPTER__ABSTRACT_ROBOT_CLIENT_HPP_

/// @file AbstractRobotClient.hpp
/// @brief Goal 7 — The API Seam.
///
/// Defines the pure-virtual interface that hides transport details from the
/// FleetAdapterNode.  Concrete implementations can be swapped without
/// modifying any RMF coordination logic:
///
///   ┌────────────────────────────────────────────┐
///   │          FleetAdapterNode (RMF)            │
///   │   calls AbstractRobotClient interface only │
///   └──────────────────┬─────────────────────────┘
///                      │  virtual dispatch
///              ┌────────┴────────┐
///              │                 │
///   ┌──────────▼──────┐  ┌──────▼──────────────┐
///   │  RosTopicClient  │  │  MockRobotClient    │
///   │  (Topics/Service)│  │  (unit test stub)   │
///   └──────────────────┘  └─────────────────────┘
///
/// Future implementations could add:
///   - HttpPollingClient  (REST API polling)
///   - GrpcClient         (gRPC streaming)

#include <string>
#include <functional>
#include <optional>
#include <array>

namespace dynominion_fleet_adapter
{

// ─────────────────────────────────────────────────────────────────────────────
// RobotStatusReport — snapshot of robot state returned by get_state()
// ─────────────────────────────────────────────────────────────────────────────

struct RobotStatusReport
{
  /// Current pose in RMF map frame [x, y, yaw]. Empty if not yet localized.
  std::optional<std::array<double, 3>> pose;

  /// Battery state-of-charge in [0.0, 1.0].
  double battery_soc = 1.0;

  /// True if the robot is currently executing a task.
  bool is_navigating = false;

  /// True if the robot is in an error/fault state.
  bool is_error = false;

  /// Human-readable description of any active error.
  std::string error_reason;
};

// ─────────────────────────────────────────────────────────────────────────────
// AbstractRobotClient
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Goal 7 — Pure-virtual transport interface.
///
/// The FleetAdapterNode calls ONLY these methods to interact with a robot.
/// This decouples RMF coordination from any specific transport (ROS topics,
/// HTTP, gRPC, etc.).
class AbstractRobotClient
{
public:
  /// Callback type invoked when a navigation command completes.
  /// @param success  true = arrived; false = aborted/failed.
  using NavigationResultCallback = std::function<void(bool /*success*/)>;

  virtual ~AbstractRobotClient() = default;

  // ── Command interface ────────────────────────────────────────────────────

  /// Command the robot to navigate to a map-frame pose.
  ///
  /// @param x, y, yaw      Target pose.
  /// @param waypoint_name  Optional semantic name (for logging / diagnostics).
  /// @param task_id        Unique task identifier.
  /// @param on_result      Callback invoked when navigation finishes.
  /// @returns true if the command was dispatched; false if rejected (e.g. busy).
  virtual bool navigate(
    double x, double y, double yaw,
    const std::string & waypoint_name,
    const std::string & task_id,
    NavigationResultCallback on_result) = 0;

  /// Command the robot to stop immediately.
  virtual void stop() = 0;

  // ── Status polling interface ─────────────────────────────────────────────

  /// Poll the robot for a synchronous state snapshot.
  /// Called by the FleetAdapterNode at a controlled rate (e.g. 5 Hz) to
  /// update RMF's traffic schedule without flooding the scheduler.
  virtual RobotStatusReport get_state() = 0;

  // ── Recovery interface ───────────────────────────────────────────────────

  /// Request the robot to clear its error state and return to IDLE.
  /// No-op if the robot is not in an error state.
  virtual void recover() = 0;

  // ── Identity ─────────────────────────────────────────────────────────────

  /// @returns robot name this client is bound to.
  virtual const std::string & robot_name() const = 0;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__ABSTRACT_ROBOT_CLIENT_HPP_
