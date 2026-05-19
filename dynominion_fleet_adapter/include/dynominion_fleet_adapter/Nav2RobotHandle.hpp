#ifndef DYNOMINION_FLEET_ADAPTER__NAV2ROBOTHANDLE_HPP_
#define DYNOMINION_FLEET_ADAPTER__NAV2ROBOTHANDLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2/utils.h>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <optional>

class Nav2RobotHandle
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2RobotHandle(
    const std::string& name,
    const rclcpp::Node::SharedPtr& node);

  const std::string& name() const { return name_; }

  void set_update_handle(const std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle>& update_handle);

  // New methods for event-based initialization
  bool is_ready();
  rmf_fleet_adapter::agv::EasyFullControl::RobotState get_state();
  void set_level_name(const std::string& level_name) { level_name_ = level_name; }

  // Goal 6/7: update cached state from Manager poll
  void update_state(const Eigen::Vector3d& pose, double battery, bool localized);

  // Callbacks for RMF EasyFullControl (Jazzy API)
  void navigate(
    rmf_fleet_adapter::agv::EasyFullControl::Destination destination,
    rmf_fleet_adapter::agv::EasyFullControl::CommandExecution execution);

  void stop(rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr identifier);

private:
  std::string name_;
  std::string level_name_ = "L1";
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle> update_handle_;
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Cached state from Process 2 (FleetManagerNode)
  Eigen::Vector3d cached_pose_ = Eigen::Vector3d::Zero();
  double cached_battery_ = 1.0;
  bool is_localized_ = false;

  std::unique_ptr<rmf_fleet_adapter::agv::EasyFullControl::CommandExecution> active_execution_;
  GoalHandleNav::SharedPtr active_goal_handle_;
};

#endif  // DYNOMINION_FLEET_ADAPTER__NAV2ROBOTHANDLE_HPP_
