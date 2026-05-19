#ifndef DYNOMINION_FLEET_ADAPTER__HTTP_ROBOT_CLIENT_HPP_
#define DYNOMINION_FLEET_ADAPTER__HTTP_ROBOT_CLIENT_HPP_

/// @file HttpRobotClient.hpp
/// @brief Concrete AbstractRobotClient implementation using REST API.

#include <dynominion_fleet_adapter/AbstractRobotClient.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>

// Forward declaration to hide httplib from the header
namespace httplib { class Client; }

namespace dynominion_fleet_adapter
{

class HttpRobotClient : public AbstractRobotClient
{
public:
  /// @param node         Shared ROS2 node (for logging).
  /// @param robot_name   Robot namespace (e.g. "dynominion1").
  /// @param host         The HTTP server host (e.g. "localhost").
  /// @param port         The HTTP server port (e.g. 8080).
  HttpRobotClient(
    rclcpp::Node::SharedPtr node,
    const std::string & robot_name,
    const std::string & host,
    int port);

  ~HttpRobotClient() override;

  // ── AbstractRobotClient interface ─────────────────────────────────────────

  bool navigate(
    double x, double y, double yaw,
    const std::string & waypoint_name,
    const std::string & task_id,
    NavigationResultCallback on_result) override;

  void stop() override;

  RobotStatusReport get_state() override;

  void recover() override;

  const std::string & robot_name() const override { return robot_name_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string robot_name_;
  
  std::unique_ptr<httplib::Client> http_client_;

  // Pending navigation result — detected via polling in get_state()
  std::string pending_task_id_;
  bool task_started_ = false;
  NavigationResultCallback pending_result_cb_;
};

}  // namespace dynominion_fleet_adapter

#endif  // DYNOMINION_FLEET_ADAPTER__HTTP_ROBOT_CLIENT_HPP_
