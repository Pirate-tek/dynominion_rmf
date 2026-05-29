#ifndef DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_
#define DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_

/// @file FleetManagerNode.hpp
/// @brief Fleet Manager Node — Process 2 of 2.
///
/// Owns all low-level robot interaction:
///   - NavController   (waypoint sequencer, 50 Hz control loop)
///   - Nav2Handler     (NavigateToPose action client)
///   - RobotStateMachine (lifecycle state tracking)
///
/// Exposes a ROS2 Action server per robot:
///   Action: /<robot_name>/navigate_robot  (NavigateRobot.action)
///   - Goal:     target pose + task_id (sent by FleetAdapterNode)
///   - Feedback: live pose + battery    (streamed during navigation)
///   - Result:   success / error_reason (fired on completion — zero polling)
///
/// Architecture: Two-process decomposition.
/// A crash here (and its Nav2 dependency) does NOT take down FleetAdapterNode,
/// allowing RMF to maintain the robot's ghost footprint in the traffic schedule.

#include <dynominion_fleet_adapter/NavController.hpp>
#include <dynominion_fleet_adapter/Nav2Handler.hpp>
#include <dynominion_fleet_adapter/RMFHandler.hpp>
#include <dynominion_fleet_adapter/FleetValidator.hpp>
#include <dynominion_fleet_adapter/RobotStateMachine.hpp>

// Generated ROS2 action type
#include <dynominion_fleet_adapter/action/navigate_robot.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace dynominion_fleet_adapter
{

using NavigateRobot = dynominion_fleet_adapter::action::NavigateRobot;
using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateRobot>;

// ─────────────────────────────────────────────────────────────────────────────
// Per-robot context owned by FleetManagerNode
// ─────────────────────────────────────────────────────────────────────────────

struct ManagedRobotContext
{
  std::string name;

  std::shared_ptr<RMFHandler>         rmf_handler;
  std::shared_ptr<Nav2Handler>        nav2_handler;
  std::shared_ptr<NavController>      nav_controller;
  std::shared_ptr<RobotStateMachine>  state_machine;

  // Telemetry subscriptions — AMCL pose + battery (on-robot topics)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr                battery_sub;

  // Cached battery (updated by battery_sub, read when building feedback)
  double latest_battery_soc{1.0};

  // ── ROS2 Action server (replaces HTTP /navigate + /state polling) ─────────
  rclcpp_action::Server<NavigateRobot>::SharedPtr action_server;

  // Active goal handle — set when a goal is accepted, cleared on result
  std::shared_ptr<NavigateGoalHandle> current_goal_handle;
  std::mutex                          goal_handle_mtx;
};

// ─────────────────────────────────────────────────────────────────────────────
// FleetManagerNode
// ─────────────────────────────────────────────────────────────────────────────

class FleetManagerNode : public rclcpp::Node
{
public:
  explicit FleetManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FleetManagerNode() = default;

  void init();

private:
  /// Build one ManagedRobotContext per robot declared in the config.
  void setup_robot(const std::string & robot_name, const std::string & fleet_name);

  /// Create the NavigateRobot action server for a single robot.
  void setup_action_server(const std::shared_ptr<ManagedRobotContext> & ctx);

  std::shared_ptr<FleetValidator> validator_;
  std::unordered_map<std::string, std::shared_ptr<ManagedRobotContext>> robots_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_
