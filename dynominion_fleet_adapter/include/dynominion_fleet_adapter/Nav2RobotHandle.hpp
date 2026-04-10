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

  void set_update_handle(const std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle>& update_handle);

  // Callbacks for RMF EasyFullControl (Jazzy API)
  void navigate(
    rmf_fleet_adapter::agv::EasyFullControl::Destination destination,
    rmf_fleet_adapter::agv::EasyFullControl::CommandExecution execution);

  void stop(rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr identifier);

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void update_loop();

  std::string name_;
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle> update_handle_;
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  double last_battery_soc_ = 1.0;

  std::unique_ptr<rmf_fleet_adapter::agv::EasyFullControl::CommandExecution> active_execution_;
  GoalHandleNav::SharedPtr active_goal_handle_;
};

#endif  // DYNOMINION_FLEET_ADAPTER__NAV2ROBOTHANDLE_HPP_
