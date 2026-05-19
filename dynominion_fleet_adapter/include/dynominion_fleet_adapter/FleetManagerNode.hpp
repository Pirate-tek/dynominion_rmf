#ifndef DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_
#define DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_

/// @file FleetManagerNode.hpp
/// @brief Goal 6 — Fleet Manager Node (Process 2 of 2).
///
/// Owns all low-level robot interaction:
///   - NavController   (waypoint sequencer, control loop)
///   - Nav2Handler     (NavigateToPose action client)
///   - RobotStateMachine (lifecycle state tracking)
///
/// Exposes a Web API (REST) for the FleetAdapterNode to poll and command:
///   GET  /v1/robots/<robot>/state
///   POST /v1/robots/<robot>/navigate
///   POST /v1/robots/<robot>/stop
///   POST /v1/robots/<robot>/recover
///
/// Subscribes to path_request topics published by FleetAdapterNode for
/// navigation commands (existing "Direct Injection" transport).
///
/// Architecture: Two-process decomposition (Goal 6).
/// A crash in this node (and its Nav2 dependency) does NOT take down the
/// FleetAdapterNode, allowing RMF to maintain the robot's ghost footprint
/// in the traffic schedule and prevent collisions.

#include <dynominion_fleet_adapter/NavController.hpp>
#include <dynominion_fleet_adapter/Nav2Handler.hpp>
#include <dynominion_fleet_adapter/RMFHandler.hpp>
#include <dynominion_fleet_adapter/FleetValidator.hpp>
#include <dynominion_fleet_adapter/RobotStateMachine.hpp>

// Web API server
#include <dynominion_fleet_adapter/thirdparty/httplib.h>
#include <dynominion_fleet_adapter/thirdparty/json.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace dynominion_fleet_adapter
{

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

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;

  // HTTP API handles polling and commands now.
};

// ─────────────────────────────────────────────────────────────────────────────
// FleetManagerNode
// ─────────────────────────────────────────────────────────────────────────────

class FleetManagerNode : public rclcpp::Node
{
public:
  explicit FleetManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FleetManagerNode();

  void init();

  /// Build one ManagedRobotContext per robot declared in the config.
  void setup_robot(const std::string & robot_name, const std::string & fleet_name);

  /// Setup the HTTP Server
  void setup_http_server();

  std::shared_ptr<FleetValidator> validator_;
  std::unordered_map<std::string, std::shared_ptr<ManagedRobotContext>> robots_;

  httplib::Server http_server_;
  std::thread http_thread_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__FLEET_MANAGER_NODE_HPP_
