#include <dynominion_fleet_adapter/FleetManagerNode.hpp>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dynominion_fleet_adapter
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

FleetManagerNode::FleetManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fleet_manager_node", options)
{
}

void FleetManagerNode::init()
{
  RCLCPP_INFO(get_logger(), "[FleetManagerNode] Starting up...");

  // ── Fleet Identity Validation ─────────────────────────────────────────────
  validator_ = std::make_shared<FleetValidator>(shared_from_this());
  if (!validator_->is_valid())
  {
    RCLCPP_FATAL(get_logger(),
      "[FleetManagerNode] FleetValidator startup check FAILED. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  const std::string fleet_name = validator_->fleet_name();

  // ── Load robot list from config ───────────────────────────────────────────
  declare_parameter("config_file", "");
  const std::string config_file = get_parameter("config_file").as_string();

  if (config_file.empty())
  {
    RCLCPP_ERROR(get_logger(),
      "[FleetManagerNode] config_file parameter is empty!");
    return;
  }

  YAML::Node config;
  try { config = YAML::LoadFile(config_file); }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(),
      "[FleetManagerNode] Failed to load YAML '%s': %s",
      config_file.c_str(), e.what());
    return;
  }

  auto robots_node = config["rmf_fleet"]["robots"];
  if (!robots_node)
  {
    RCLCPP_ERROR(get_logger(),
      "[FleetManagerNode] Config missing 'rmf_fleet.robots' section!");
    return;
  }

  for (auto it = robots_node.begin(); it != robots_node.end(); ++it)
  {
    const std::string robot_name = it->first.as<std::string>();
    setup_robot(robot_name, fleet_name);
  }

  RCLCPP_INFO(get_logger(),
    "[FleetManagerNode] Initialised. Managing %zu robot(s) in fleet '%s'. "
    "Action servers ready on /<robot>/navigate_robot.",
    robots_.size(), fleet_name.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-robot setup
// ─────────────────────────────────────────────────────────────────────────────

void FleetManagerNode::setup_robot(
  const std::string & robot_name,
  const std::string & fleet_name)
{
  auto ctx = std::make_shared<ManagedRobotContext>();
  ctx->name = robot_name;

  // ── State Machine ─────────────────────────────────────────────────────────
  ctx->state_machine = std::make_shared<RobotStateMachine>(robot_name, get_logger());
  ctx->state_machine->set_transition_callback(
    [this, robot_name](RobotState from, RobotState to, const std::string & ctx_msg)
    {
      RCLCPP_INFO(get_logger(),
        "[FleetManagerNode][%s] State: %s → %s  (%s)",
        robot_name.c_str(), to_string(from), to_string(to), ctx_msg.c_str());
    });

  // ── RMFHandler ────────────────────────────────────────────────────────────
  ctx->rmf_handler = std::make_shared<RMFHandler>(shared_from_this(), validator_, robot_name);

  // ── Nav2Handler ───────────────────────────────────────────────────────────
  ctx->nav2_handler = std::make_shared<Nav2Handler>(shared_from_this(), robot_name);

  // ── Telemetry: AMCL pose ──────────────────────────────────────────────────
  ctx->pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/" + robot_name + "/amcl_pose", rclcpp::QoS(10).transient_local(),
    [ctx](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      const auto & p = msg->pose.pose.position;
      const auto & o = msg->pose.pose.orientation;
      tf2::Quaternion q;
      tf2::fromMsg(o, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      ctx->rmf_handler->update_pose(p.x, p.y, yaw);

      if (ctx->nav_controller)
        ctx->nav_controller->update_pose(p.x, p.y, yaw);
    });

  // ── Telemetry: battery ────────────────────────────────────────────────────
  ctx->battery_sub = create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + robot_name + "/battery_state", 10,
    [ctx](const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
      ctx->rmf_handler->update_battery(msg->percentage);
      ctx->latest_battery_soc = msg->percentage;
    });

  // ── NavController ─────────────────────────────────────────────────────────
  ctx->nav_controller = std::make_shared<NavController>(
    shared_from_this(), ctx->rmf_handler, ctx->nav2_handler,
    robot_name, fleet_name);

  // Wire NavController state hooks → state machine
  ctx->nav_controller->set_on_task_started(
    [ctx](const std::string & task_id)
    {
      ctx->state_machine->on_navigate(task_id);
    });

  ctx->nav_controller->set_on_task_accepted(
    [ctx](const std::string & /*task_id*/)
    {
      ctx->state_machine->on_navigate_accepted();
    });

  // Wire NavController completion → action result (event-driven, zero poll lag)
  ctx->nav_controller->set_on_task_finished(
    [ctx](bool success, const std::string & reason)
    {
      if (success)
        ctx->state_machine->on_success();
      else
        ctx->state_machine->on_error(reason);

      std::lock_guard<std::mutex> lk(ctx->goal_handle_mtx);
      if (!ctx->current_goal_handle)
        return;

      auto result = std::make_shared<NavigateRobot::Result>();
      result->success      = success;
      result->error_reason = reason;

      if (success)
      {
        RCLCPP_INFO(rclcpp::get_logger("fleet_manager_node"),
          "[FleetManagerNode][%s] Task finished successfully — sending action result.",
          ctx->name.c_str());
        ctx->current_goal_handle->succeed(result);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("fleet_manager_node"),
          "[FleetManagerNode][%s] Task aborted: %s — sending action abort.",
          ctx->name.c_str(), reason.c_str());
        ctx->current_goal_handle->abort(result);
      }
      ctx->current_goal_handle = nullptr;
    });

  // Wire NavController pose updates → action feedback
  ctx->nav_controller->set_on_pose_update(
    [ctx](double x, double y, double yaw)
    {
      std::lock_guard<std::mutex> lk(ctx->goal_handle_mtx);
      if (!ctx->current_goal_handle)
        return;
      if (ctx->current_goal_handle->is_canceling())
        return;

      auto feedback = std::make_shared<NavigateRobot::Feedback>();
      feedback->pose_x       = x;
      feedback->pose_y       = y;
      feedback->pose_yaw     = yaw;
      feedback->battery_soc  = ctx->latest_battery_soc;
      ctx->current_goal_handle->publish_feedback(feedback);
    });

  // ── Action server ─────────────────────────────────────────────────────────
  setup_action_server(ctx);

  RCLCPP_INFO(get_logger(),
    "[FleetManagerNode] Robot [%s] ready. Action server: /%s/navigate_robot",
    robot_name.c_str(), robot_name.c_str());

  robots_.emplace(robot_name, ctx);
}

// ─────────────────────────────────────────────────────────────────────────────
// Action server — one per robot, replaces the entire HTTP server
// ─────────────────────────────────────────────────────────────────────────────

void FleetManagerNode::setup_action_server(const std::shared_ptr<ManagedRobotContext> & ctx)
{
  const std::string action_name = "/" + ctx->name + "/navigate_robot";

  ctx->action_server = rclcpp_action::create_server<NavigateRobot>(
    this, action_name,

    // ── handle_goal: always accept ───────────────────────────────────────────
    [ctx](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateRobot::Goal> goal)
      -> rclcpp_action::GoalResponse
    {
      RCLCPP_INFO(rclcpp::get_logger("fleet_manager_node"),
        "[FleetManagerNode][%s] Goal received: task='%s' target=(%.2f, %.2f, %.2f).",
        ctx->name.c_str(), goal->task_id.c_str(), goal->x, goal->y, goal->yaw);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },

    // ── handle_cancel: honour cancel → NavController.cancel_all() ──────────
    [ctx](std::shared_ptr<NavigateGoalHandle> /*goal_handle*/)
      -> rclcpp_action::CancelResponse
    {
      RCLCPP_INFO(rclcpp::get_logger("fleet_manager_node"),
        "[FleetManagerNode][%s] Cancel requested.", ctx->name.c_str());
      ctx->nav_controller->cancel_all();
      return rclcpp_action::CancelResponse::ACCEPT;
    },

    // ── handle_accepted: store handle, kick off navigation ──────────────────
    [ctx](std::shared_ptr<NavigateGoalHandle> goal_handle)
    {
      const auto & goal = goal_handle->get_goal();

      {
        std::lock_guard<std::mutex> lk(ctx->goal_handle_mtx);
        // Preempt any previous goal that was never properly closed
        if (ctx->current_goal_handle)
        {
          RCLCPP_WARN(rclcpp::get_logger("fleet_manager_node"),
            "[FleetManagerNode][%s] Preempting previous goal.", ctx->name.c_str());
          auto abort_result = std::make_shared<NavigateRobot::Result>();
          abort_result->success = false;
          abort_result->error_reason = "preempted by new goal";
          ctx->current_goal_handle->abort(abort_result);
        }
        ctx->current_goal_handle = goal_handle;
      }

      RCLCPP_INFO(rclcpp::get_logger("fleet_manager_node"),
        "[FleetManagerNode][%s] Executing goal: task='%s' (%.2f, %.2f, %.2f).",
        ctx->name.c_str(), goal->task_id.c_str(), goal->x, goal->y, goal->yaw);

      // Start navigation — completion fires through set_on_task_finished hook
      ctx->nav_controller->navigate(goal->x, goal->y, goal->yaw, goal->task_id);
    });

  RCLCPP_INFO(get_logger(),
    "[FleetManagerNode] Action server created: %s", action_name.c_str());
}

}  // namespace dynominion_fleet_adapter

// ─────────────────────────────────────────────────────────────────────────────
// main — fleet_manager_node executable
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "[fleet_manager_node] Starting Fleet Manager Node (ROS2 Action API)...");

  auto node = std::make_shared<dynominion_fleet_adapter::FleetManagerNode>();
  node->init();

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "[fleet_manager_node] Shutting down.");
  rclcpp::shutdown();
  return 0;
}
