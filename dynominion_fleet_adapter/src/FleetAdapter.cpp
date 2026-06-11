#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>
#include <dynominion_fleet_adapter/Nav2RobotHandle.hpp>
#include <dynominion_fleet_adapter/FleetValidator.hpp>
#include <dynominion_fleet_adapter/RMFHandler.hpp>
#include <dynominion_fleet_adapter/action/navigate_robot.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <Eigen/Dense>
#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>
#include <optional>
#include <mutex>

using namespace rmf_fleet_adapter::agv;
using NavigateRobot = dynominion_fleet_adapter::action::NavigateRobot;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateRobot>;

// ─────────────────────────────────────────────────────────────────────────────
// Coordinate transformation helper (unchanged)
// ─────────────────────────────────────────────────────────────────────────────

Transformation compute_transformation(
  const std::string & level,
  const YAML::Node & coords,
  const rclcpp::Logger & logger)
{
  auto rmf_node   = coords["rmf"];
  auto robot_node = coords["robot"];

  if (!rmf_node || !robot_node || rmf_node.size() < 2 || robot_node.size() < 2)
  {
    RCLCPP_WARN(logger, "Not enough points for transformation on level [%s].", level.c_str());
    return Transformation(0.0, 1.0, Eigen::Vector2d::Zero());
  }

  int n = std::min((int)rmf_node.size(), (int)robot_node.size());
  double sum_x = 0, sum_y = 0, sum_xp = 0, sum_yp = 0;
  for (int i = 0; i < n; ++i)
  {
    sum_x  += rmf_node[i][0].as<double>();
    sum_y  += rmf_node[i][1].as<double>();
    sum_xp += robot_node[i][0].as<double>();
    sum_yp += robot_node[i][1].as<double>();
  }
  double mean_x  = sum_x / n,  mean_y  = sum_y / n;
  double mean_xp = sum_xp / n, mean_yp = sum_yp / n;

  double s_uu_vv = 0, s_uup_vvp = 0, s_uvp_vup = 0;
  for (int i = 0; i < n; ++i)
  {
    double u  = rmf_node[i][0].as<double>()  - mean_x;
    double v  = rmf_node[i][1].as<double>()  - mean_y;
    double up = robot_node[i][0].as<double>() - mean_xp;
    double vp = robot_node[i][1].as<double>() - mean_yp;
    s_uu_vv   += u*u + v*v;
    s_uup_vvp += u*up + v*vp;
    s_uvp_vup += u*vp - v*up;
  }

  if (s_uu_vv < 1e-9)
    return Transformation(0.0, 1.0, Eigen::Vector2d(mean_xp - mean_x, mean_yp - mean_y));

  double a  = s_uup_vvp / s_uu_vv;
  double b  = s_uvp_vup / s_uu_vv;
  double tx = mean_xp - (a * mean_x - b * mean_y);
  double ty = mean_yp - (b * mean_x + a * mean_y);

  RCLCPP_INFO(logger, "Computed transformation for [%s]: rot=%.3f, scale=%.3f, trans=(%.3f, %.3f)",
    level.c_str(), std::atan2(b, a), std::sqrt(a*a + b*b), tx, ty);

  return Transformation(std::atan2(b, a), std::sqrt(a*a + b*b), Eigen::Vector2d(tx, ty));
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-robot context
// ─────────────────────────────────────────────────────────────────────────────

struct RobotContext
{
  std::string name;

  std::shared_ptr<dynominion_fleet_adapter::RMFHandler>  rmf_handler;
  std::shared_ptr<Nav2RobotHandle>                       nav_handle;
  std::shared_ptr<EasyFullControl::EasyRobotUpdateHandle> update_handle;

  bool registered = false;

  EasyFullControl::ConstActivityIdentifierPtr current_activity;

  // ── ROS2 Action client ────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateRobot>::SharedPtr action_client;
  NavGoalHandle::SharedPtr                        current_goal_handle;
  std::mutex                                      goal_handle_mtx;

  // ── Direct telemetry subscriptions ────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr                battery_sub;
  double latest_battery{1.0};
  Eigen::Vector3d latest_pose{0.0, 0.0, 0.0};
  bool pose_received{false};
};

// ─────────────────────────────────────────────────────────────────────────────
// FleetAdapterNode
// ─────────────────────────────────────────────────────────────────────────────

class FleetAdapterNode
{
public:
  FleetAdapterNode(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<Adapter> & adapter)
  : node_(node), adapter_(adapter) {}

  void init()
  {
    validator_ = std::make_shared<dynominion_fleet_adapter::FleetValidator>(node_);
    if (!validator_->is_valid())
    {
      RCLCPP_FATAL(node_->get_logger(), "[FleetAdapterNode] FleetValidator FAILED.");
      rclcpp::shutdown(); return;
    }

    node_->declare_parameter("config_file", "");
    node_->declare_parameter("nav_graph_path", "");

    const std::string config_file = node_->get_parameter("config_file").as_string();
    if (config_file.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "config_file parameter is empty!"); return;
    }

    YAML::Node config;
    try { config = YAML::LoadFile(config_file); }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML: %s", e.what()); return;
    }

    auto rmf_fleet_node = config["rmf_fleet"];
    if (!rmf_fleet_node)
    {
      RCLCPP_ERROR(node_->get_logger(), "Config missing 'rmf_fleet' section!"); return;
    }

    const std::string fleet_name = validator_->fleet_name();

    // Vehicle traits
    auto limits_node  = rmf_fleet_node["limits"];
    auto profile_node = rmf_fleet_node["profile"];
    auto footprint = rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
      profile_node["footprint_radius"].as<double>());
    auto traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
      rmf_traffic::agv::VehicleTraits::Limits{
        limits_node["max_linear_speed"].as<double>(),
        limits_node["max_linear_acceleration"].as<double>()},
      rmf_traffic::agv::VehicleTraits::Limits{
        limits_node["max_angular_speed"].as<double>(),
        limits_node["max_angular_acceleration"].as<double>()},
      rmf_traffic::Profile(footprint));
    traits->set_differential(
      rmf_traffic::agv::VehicleTraits::Differential(Eigen::Vector2d::UnitX(), true));

    // Nav graph
    const std::string graph_path = node_->get_parameter("nav_graph_path").as_string();
    if (graph_path.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "nav_graph_path parameter is empty!"); return;
    }
    auto graph = std::make_shared<rmf_traffic::agv::Graph>(parse_graph(graph_path, *traits));

    // Robot configs
    auto robots_node = rmf_fleet_node["robots"];
    if (robots_node)
    {
      for (auto it = robots_node.begin(); it != robots_node.end(); ++it)
      {
        std::string r_name  = it->first.as<std::string>();
        std::string charger = it->second["charger"].as<std::string>();
        robot_configs_.emplace(r_name, EasyFullControl::RobotConfiguration({charger}));
      }
    }

    // Battery models
    auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
      rmf_battery::agv::BatterySystem::make(24.0, 40.0, 2.0).value());
    auto mechanical_system = rmf_battery::agv::MechanicalSystem::make(20.0, 10.0, 0.22).value();
    auto power_system      = rmf_battery::agv::PowerSystem::make(10.0).value();
    auto motion_sink  = std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(*battery_system, mechanical_system);
    auto ambient_sink = std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(*battery_system, power_system);

    // Task consideration
    std::unordered_map<std::string, FleetUpdateHandle::ConsiderRequest> task_consideration;
    for (const auto & task_type : validator_->supported_tasks())
      task_consideration[task_type] = consider_all();

    // Fleet config
    EasyFullControl::FleetConfiguration fleet_config(
      fleet_name, std::nullopt, robot_configs_, traits, graph,
      battery_system, motion_sink, ambient_sink, ambient_sink,
      0.2, 0.9, true, task_consideration, {}, nullptr, true,
      std::nullopt, std::chrono::seconds(10), std::chrono::seconds(1),
      false, 2.0, 2.0, 0.3);

    auto ref_coords = config["reference_coordinates"];
    if (ref_coords)
    {
      for (auto it = ref_coords.begin(); it != ref_coords.end(); ++it)
      {
        std::string level = it->first.as<std::string>();
        Transformation tf = compute_transformation(level, it->second, node_->get_logger());
        fleet_config.add_robot_coordinate_transformation(level, tf);
      }
    }

    easy_fleet_ = adapter_->add_easy_fleet(fleet_config);
    if (!easy_fleet_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to add easy fleet!"); return;
    }

    easy_fleet_->more()->set_task_planner_params(
      battery_system, motion_sink, ambient_sink, ambient_sink,
      0.2, 0.9, true, nullptr);

    setup_fleet_event_subscriptions();
    build_robot_contexts();

    // Registration timer — checks pose arrival from AMCL subscription
    registration_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FleetAdapterNode::check_registration, this));
  }

private:

  // ── Build per-robot contexts ──────────────────────────────────────────────

  void build_robot_contexts()
  {
    for (const auto & [name, cfg] : robot_configs_)
    {
      auto ctx = std::make_shared<RobotContext>();
      ctx->name = name;

      ctx->rmf_handler = std::make_shared<dynominion_fleet_adapter::RMFHandler>(
        node_, validator_, name);

      // Action client → FleetManagerNode's action server on same DDS domain
      // In real deployment: same DDS over WiFi/LAN — no code change needed
      const std::string action_name = "/" + name + "/navigate_robot";
      ctx->action_client = rclcpp_action::create_client<NavigateRobot>(node_, action_name);

      ctx->nav_handle = std::make_shared<Nav2RobotHandle>(name, node_);

      // Direct AMCL subscription — replaces HTTP poll for telemetry + registration
      auto ctx_cap = ctx;
      ctx->amcl_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/" + name + "/amcl_pose", rclcpp::QoS(10).transient_local(),
        [ctx_cap, this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
          tf2::Quaternion q;
          tf2::fromMsg(msg->pose.pose.orientation, q);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

          ctx_cap->latest_pose = Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
          ctx_cap->pose_received = true;

          ctx_cap->nav_handle->update_state(ctx_cap->latest_pose, ctx_cap->latest_battery, true);

          // Push telemetry to RMF when idle (no active action)
          if (ctx_cap->registered && ctx_cap->update_handle)
          {
            EasyFullControl::RobotState state("L1", ctx_cap->latest_pose, ctx_cap->latest_battery);
            ctx_cap->update_handle->update(state, ctx_cap->current_activity);
          }
        });

      ctx->battery_sub = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/" + name + "/battery_state", 10,
        [ctx_cap](const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
          ctx_cap->latest_battery = msg->percentage;
        });

      robot_contexts_.emplace(name, ctx);
    }
  }

  // ── Registration timer — triggered by AMCL topic, no HTTP ─────────────────

  void check_registration()
  {
    bool all_done = true;
    for (auto & [name, ctx] : robot_contexts_)
    {
      if (ctx->registered) continue;
      all_done = false;

      if (!ctx->pose_received)
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "[FleetAdapterNode] Waiting for AMCL pose for robot [%s]...", name.c_str());
        continue;
      }

      if (!ctx->nav_handle->is_ready())
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "[FleetAdapterNode] Waiting for robot [%s] to become ready.", name.c_str());
        continue;
      }

      // Wait for action server to be available
      if (!ctx->action_client->action_server_is_ready())
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "[FleetAdapterNode] Waiting for action server [%s/navigate_robot]...", name.c_str());
        continue;
      }

      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] Robot [%s] ready — registering.", name.c_str());

      auto ctx_cap  = ctx;
      auto node_cap = node_;

      EasyFullControl::RobotCallbacks callbacks(
        // ── navigate_to_waypoint ─────────────────────────────────────────────
        [ctx_cap, node_cap](auto dest, auto exec)
        {
          ctx_cap->current_activity = exec.identifier();
          auto opt_task_id = ctx_cap->rmf_handler->on_navigate(dest);
          if (!opt_task_id.has_value()) return;

          const auto pos = dest.position();
          const std::string task_id = opt_task_id.value();

          RCLCPP_INFO(node_cap->get_logger(),
            "[FleetAdapterNode][%s] RMF → navigate to '%s' (%.2f, %.2f)",
            ctx_cap->name.c_str(), dest.name().c_str(), pos.x(), pos.y());

          // Build action goal
          auto goal = NavigateRobot::Goal{};
          goal.x       = pos.x();
          goal.y       = pos.y();
          goal.yaw     = dest.yaw();
          goal.task_id = task_id;

          // We wrap the CommandExecution in a thread-safe shared wrapper to ensure
          // finished() is called exactly once, whether accepted or rejected.
          auto shared_exec = std::make_shared<std::optional<rmf_fleet_adapter::agv::EasyFullControl::CommandExecution>>(std::move(exec));
          auto exec_mutex = std::make_shared<std::mutex>();
          auto finish_exec = [shared_exec, exec_mutex]() {
            std::lock_guard<std::mutex> lock(*exec_mutex);
            if (shared_exec->has_value())
            {
              (*shared_exec)->finished();
              shared_exec->reset();
            }
          };

          auto options = rclcpp_action::Client<NavigateRobot>::SendGoalOptions{};

          // Goal accepted/rejected callback
          options.goal_response_callback =
            [ctx_cap, finish_exec](const NavGoalHandle::SharedPtr & handle)
            {
              if (!handle)
              {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                  "[FleetAdapterNode][%s] Goal was REJECTED by Fleet Manager action server.",
                  ctx_cap->name.c_str());
                finish_exec();
              }
              else
              {
                std::lock_guard<std::mutex> lk(ctx_cap->goal_handle_mtx);
                ctx_cap->current_goal_handle = handle;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "[FleetAdapterNode][%s] Goal accepted by Fleet Manager.",
                  ctx_cap->name.c_str());
              }
            };

          // Feedback → push live telemetry to RMF (event-driven, no poll)
          options.feedback_callback =
            [ctx_cap](NavGoalHandle::SharedPtr, const std::shared_ptr<const NavigateRobot::Feedback> fb)
            {
              Eigen::Vector3d pose(fb->pose_x, fb->pose_y, fb->pose_yaw);
              ctx_cap->latest_pose    = pose;
              ctx_cap->latest_battery = fb->battery_soc;
              ctx_cap->nav_handle->update_state(pose, fb->battery_soc, true);

              if (ctx_cap->update_handle)
              {
                EasyFullControl::RobotState state("L1", pose, fb->battery_soc);
                ctx_cap->update_handle->update(state, ctx_cap->current_activity);
              }
            };

          // Result → exec.finished() called instantly on task completion
          options.result_callback =
            [ctx_cap, finish_exec](const NavGoalHandle::WrappedResult & result)
            {
              std::lock_guard<std::mutex> lk(ctx_cap->goal_handle_mtx);
              ctx_cap->current_goal_handle = nullptr;
              ctx_cap->current_activity    = nullptr;

              if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                  result.result->success)
              {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "[FleetAdapterNode][%s] Navigation COMPLETE — notifying RMF.",
                  ctx_cap->name.c_str());
              }
              else
              {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "[FleetAdapterNode][%s] Navigation FAILED: %s",
                  ctx_cap->name.c_str(),
                  result.result ? result.result->error_reason.c_str() : "unknown");
              }
              finish_exec();
            };

          ctx_cap->action_client->async_send_goal(goal, options);
        },

        // ── stop / interrupt ─────────────────────────────────────────────────
        [ctx_cap](auto ident)
        {
          ctx_cap->current_activity = nullptr;
          ctx_cap->rmf_handler->on_stop(ident);

          std::lock_guard<std::mutex> lk(ctx_cap->goal_handle_mtx);
          if (ctx_cap->current_goal_handle)
          {
            ctx_cap->action_client->async_cancel_goal(ctx_cap->current_goal_handle);
            ctx_cap->current_goal_handle = nullptr;
          }
        },

        // ── action_executor (custom actions) ─────────────────────────────────
        [ctx_cap, this](
          const std::string & category,
          const nlohmann::json &,
          rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution action_exec)
        {
          ctx_cap->current_activity = action_exec.identifier();
          handle_custom_action(ctx_cap, category, std::move(action_exec));
        }
      );

      auto handle = easy_fleet_->add_robot(
        name, ctx->nav_handle->get_state(), robot_configs_.at(name), callbacks);

      if (handle)
      {
        ctx->update_handle = handle;
        ctx->nav_handle->set_update_handle(handle);
        ctx->registered = true;
        RCLCPP_INFO(node_->get_logger(),
          "[FleetAdapterNode] Robot [%s] registered successfully.", name.c_str());
      }
    }

    if (all_done && registration_timer_)
    {
      registration_timer_->cancel();
      registration_timer_ = nullptr;
      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] All robots registered. "
        "Telemetry is event-driven via AMCL subscription + action feedback.");
    }
  }

  // ── Custom action handler (unchanged) ────────────────────────────────────

  using ActionExecution = rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution;

  void handle_custom_action(
    const std::shared_ptr<RobotContext> & ctx,
    const std::string & category,
    ActionExecution action_exec)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[FleetAdapterNode][%s] Custom action: '%s'", ctx->name.c_str(), category.c_str());

    if (category == "dock" || category == "docking" ||
        category == "teleoperation" || category == "teleop" ||
        category == "patrol")
    {
      action_exec.finished();
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
        "[FleetAdapterNode][%s] Unknown action '%s' — completing immediately.",
        ctx->name.c_str(), category.c_str());
      action_exec.finished();
    }
    ctx->current_activity = nullptr;
  }

  // ── Fleet event subscriptions ─────────────────────────────────────────────

  void setup_fleet_event_subscriptions()
  {
    lane_closure_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/" + validator_->fleet_name() + "/lane_closed", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          for (auto & [name, ctx] : robot_contexts_)
          {
            std::lock_guard<std::mutex> lk(ctx->goal_handle_mtx);
            if (ctx->current_goal_handle)
            {
              ctx->action_client->async_cancel_goal(ctx->current_goal_handle);
              ctx->current_goal_handle = nullptr;
            }
          }
        }
      });

    speed_limit_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/" + validator_->fleet_name() + "/speed_limited", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        RCLCPP_INFO(node_->get_logger(),
          "[FleetAdapterNode] Speed limit event: %s", msg->data ? "ACTIVE" : "CLEARED");
      });
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Adapter> adapter_;
  std::shared_ptr<EasyFullControl> easy_fleet_;
  std::unordered_map<std::string, EasyFullControl::RobotConfiguration> robot_configs_;
  std::shared_ptr<dynominion_fleet_adapter::FleetValidator> validator_;
  std::unordered_map<std::string, std::shared_ptr<RobotContext>> robot_contexts_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_closure_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr speed_limit_sub_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto adapter = Adapter::make("dynominion_fleet_adapter");
  if (!adapter)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize RMF Adapter!");
    return 1;
  }

  auto node = adapter->node();
  auto fleet_adapter_node = std::make_shared<FleetAdapterNode>(node, adapter);

  try {
    fleet_adapter_node->init();
    adapter->start();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Init exception: %s", e.what());
    return 1;
  }

  try {
    rclcpp::spin(node);
  } catch (const std::exception &) {
    while (rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();
  return 0;
}
// Entry Point of RMF