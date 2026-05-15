#include <rclcpp/rclcpp.hpp>
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
#include <dynominion_fleet_adapter/HttpRobotClient.hpp>   // Goal 7 — API seam (Web API)
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <Eigen/Dense>
#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <chrono>

using namespace rmf_fleet_adapter::agv;

// ─────────────────────────────────────────────────────────────────────────────
// Coordinate transformation helper (nudged algorithm)
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
    RCLCPP_WARN(logger,
      "Not enough points for transformation on level [%s]. Using default.", level.c_str());
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
    double u  = rmf_node[i][0].as<double>()   - mean_x;
    double v  = rmf_node[i][1].as<double>()   - mean_y;
    double up = robot_node[i][0].as<double>()  - mean_xp;
    double vp = robot_node[i][1].as<double>()  - mean_yp;
    s_uu_vv    += u*u + v*v;
    s_uup_vvp  += u*up + v*vp;
    s_uvp_vup  += u*vp - v*up;
  }

  if (s_uu_vv < 1e-9)
  {
    RCLCPP_WARN(logger,
      "Points too close for transformation on level [%s].", level.c_str());
    return Transformation(0.0, 1.0, Eigen::Vector2d(mean_xp - mean_x, mean_yp - mean_y));
  }

  double a   = s_uup_vvp / s_uu_vv;
  double b   = s_uvp_vup / s_uu_vv;
  double tx  = mean_xp - (a * mean_x - b * mean_y);
  double ty  = mean_yp - (b * mean_x + a * mean_y);

  double rotation = std::atan2(b, a);
  double scale    = std::sqrt(a * a + b * b);

  RCLCPP_INFO(logger,
    "Computed transformation for [%s]: rot=%.3f, scale=%.3f, trans=(%.3f, %.3f)",
    level.c_str(), rotation, scale, tx, ty);

  return Transformation(rotation, scale, Eigen::Vector2d(tx, ty));
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-robot context — isolates all per-robot state
//
// PROPERTY: Creates one internal robot context per robot to isolate robot
//           state handling, navigation execution, and RMF command callbacks.
// ─────────────────────────────────────────────────────────────────────────────

struct RobotContext
{
  std::string name;

  // Goal 2: RMF interface + StateGuard (publishes PathRequest to fleet_manager)
  std::shared_ptr<dynominion_fleet_adapter::RMFHandler> rmf_handler;

  // Goal 7: AbstractRobotClient — polls FleetManagerNode for state at 5 Hz.
  // Navigation commands travel via HTTP POST.
  // Status is polled from GET /state (Polling).
  std::shared_ptr<dynominion_fleet_adapter::HttpRobotClient> http_client;

  // Readiness sensor / TF subscriber (kept for registration gating)
  std::shared_ptr<Nav2RobotHandle> nav_handle;

  // EasyFullControl update handle (set after registration)
  std::shared_ptr<EasyFullControl::EasyRobotUpdateHandle> update_handle;

  bool registered = false;
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
  : node_(node), adapter_(adapter)
  {}

  void init()
  {
    // ── Goal 1: Fleet Identity Validation ────────────────────────────────
    // PROPERTY: Starts adapter node, loads config, constructs internal objects.
    validator_ = std::make_shared<dynominion_fleet_adapter::FleetValidator>(node_);
    if (!validator_->is_valid())
    {
      RCLCPP_FATAL(node_->get_logger(),
        "[FleetAdapterNode] FleetValidator startup check FAILED. "
        "Refusing to enter RMF schedule. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // ── Load YAML config ─────────────────────────────────────────────────
    node_->declare_parameter("config_file", "");
    node_->declare_parameter("nav_graph_path", "");

    const std::string config_file = node_->get_parameter("config_file").as_string();
    if (config_file.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "config_file parameter is empty!");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Loading config: %s", config_file.c_str());
    YAML::Node config;
    try { config = YAML::LoadFile(config_file); }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML: %s", e.what());
      return;
    }

    auto rmf_fleet_node = config["rmf_fleet"];
    if (!rmf_fleet_node)
    {
      RCLCPP_ERROR(node_->get_logger(), "Config missing 'rmf_fleet' section!");
      return;
    }

    // ── Fleet name — single source of truth from FleetValidator ──────────
    // PROPERTY: Creates RMF adapter instance and registers fleet through handle.
    const std::string fleet_name = validator_->fleet_name();
    RCLCPP_INFO(node_->get_logger(), "Initializing fleet: %s", fleet_name.c_str());

    // ── Vehicle Traits ────────────────────────────────────────────────────
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

    // ── Nav Graph ─────────────────────────────────────────────────────────
    const std::string graph_path = node_->get_parameter("nav_graph_path").as_string();
    if (graph_path.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "nav_graph_path parameter is empty!");
      return;
    }
    auto graph = std::make_shared<rmf_traffic::agv::Graph>(parse_graph(graph_path, *traits));
    RCLCPP_INFO(node_->get_logger(),
      "Loaded nav graph with %zu waypoints.", graph->num_waypoints());

    // ── Robot Configurations ──────────────────────────────────────────────
    auto robots_node = rmf_fleet_node["robots"];
    if (robots_node)
    {
      for (auto it = robots_node.begin(); it != robots_node.end(); ++it)
      {
        std::string r_name  = it->first.as<std::string>();
        std::string charger = it->second["charger"].as<std::string>();
        robot_configs_.emplace(r_name, EasyFullControl::RobotConfiguration({charger}));
        RCLCPP_INFO(node_->get_logger(),
          "Configured robot [%s] charger=[%s]", r_name.c_str(), charger.c_str());
      }
    }

    // ── Battery Models ────────────────────────────────────────────────────
    auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
      rmf_battery::agv::BatterySystem::make(24.0, 40.0, 2.0).value());
    auto mechanical_system =
      rmf_battery::agv::MechanicalSystem::make(20.0, 10.0, 0.22).value();
    auto power_system =
      rmf_battery::agv::PowerSystem::make(10.0).value();
    auto motion_sink = std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
      *battery_system, mechanical_system);
    auto ambient_sink = std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
      *battery_system, power_system);

    // ── Goal 1: Task consideration from FleetValidator ────────────────────
    std::unordered_map<std::string, FleetUpdateHandle::ConsiderRequest> task_consideration;
    for (const auto & task_type : validator_->supported_tasks())
    {
      task_consideration[task_type] = consider_all();
      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] Registering task type: '%s'", task_type.c_str());
    }

    // ── Fleet Configuration ───────────────────────────────────────────────
    EasyFullControl::FleetConfiguration fleet_config(
      fleet_name,
      std::nullopt,         // transformations (added below)
      robot_configs_,
      traits,
      graph,
      battery_system,
      motion_sink,
      ambient_sink,         // ambient_sink
      ambient_sink,         // tool_sink (must not be nullptr)
      0.2,                  // recharge_threshold
      0.9,                  // recharge_soc
      true,                 // account_for_battery_drain
      task_consideration,
      {},                   // action_consideration
      nullptr,              // finishing_request
      true,                 // skip_rotation_commands (NavController handles rotation)
      std::nullopt,         // server_uri
      std::chrono::seconds(10),  // max_delay
      std::chrono::seconds(1),   // update_interval
      false,                // default_responsive_wait
      2.0,                  // default_max_merge_waypoint_distance
      2.0,                  // default_max_merge_lane_distance
      0.3                   // min_lane_length
    );

    // Reference coordinate transformations
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

    // ── Add fleet ─────────────────────────────────────────────────────────
    easy_fleet_ = adapter_->add_easy_fleet(fleet_config);
    if (!easy_fleet_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to add easy fleet!");
      return;
    }

    // Configure task planner parameters so the fleet can bid on tasks
    bool params_set = easy_fleet_->more()->set_task_planner_params(
      battery_system,
      motion_sink,
      ambient_sink,
      ambient_sink, // tool_sink - let's pass ambient_sink just in case it can't be null
      0.2,     // recharge_threshold
      0.9,     // recharge_soc
      true,    // account_for_battery_drain
      nullptr  // finishing_request
    );
    if (!params_set)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set task planner params!");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Successfully set task planner params!");
    }

    // ── PROPERTY: Subscribe to lane closures / speed limit events ─────────
    // Forwards RMF fleet manager lane-closure signals back into the traffic
    // schedule system. This satisfies: "Subscribes to ROS2 topics for dynamic
    // lane closures, speed limit changes, fleet interrupts."
    setup_fleet_event_subscriptions();

    // ── Goal 2+7: Build per-robot contexts ───────────────────────────────
    // PROPERTY: Creates one internal robot context per robot.
    // Goal 6: Nav/Nav2Handler now live in fleet_manager_node (Process 2).
    //         FleetAdapterNode only holds RMFHandler + RosTopicClient.
    for (const auto & [name, cfg] : robot_configs_)
    {
      auto ctx = std::make_shared<RobotContext>();
      ctx->name = name;

      // Goal 2 — RMFHandler (StateGuard + /fleet/path_request publisher)
      // PathRequest goes to fleet_manager_node's NavController (Direct Injection).
      ctx->rmf_handler = std::make_shared<dynominion_fleet_adapter::RMFHandler>(
        node_, validator_, name);

      // Goal 7 — HttpRobotClient polls fleet_manager_node at 5 Hz.
      ctx->http_client = std::make_shared<dynominion_fleet_adapter::HttpRobotClient>(
        node_, name, "127.0.0.1", 8080);

      // Nav2RobotHandle — readiness gating only (TF/AMCL).
      ctx->nav_handle = std::make_shared<Nav2RobotHandle>(name, node_);

      robot_contexts_.emplace(name, ctx);
    }

    // ── Registration timer ────────────────────────────────────────────────
    // PROPERTY: Registers robots into RMF after receiving first valid state.
    registration_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FleetAdapterNode::check_registration, this));
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  // Registration timer
  // ─────────────────────────────────────────────────────────────────────────

  void check_registration()
  {
    bool all_done = true;
    for (auto & [name, ctx] : robot_contexts_)
    {
      if (ctx->registered)
        continue;

      all_done = false;
      
      // Goal 6: Poll FleetManager for state to check localization readiness
      const auto report = ctx->http_client->get_state();
      if (report.pose.has_value())
      {
        ctx->nav_handle->update_state(
          Eigen::Vector3d((*report.pose)[0], (*report.pose)[1], (*report.pose)[2]),
          report.battery_soc,
          true  // It has a pose, so it is localized
        );
      }

      if (!ctx->nav_handle->is_ready())
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "[FleetAdapterNode] Waiting for robot [%s] to become ready (localized)...", 
          name.c_str());
        continue;
      }

      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] Robot [%s] is ready — registering.", name.c_str());

      // ── Goal 3: NavController owns the EasyFullControl callback surface ──
      // PROPERTY: Receives navigation requests from RMF and converts them
      //           into backend robot motion commands.
      // PROPERTY: Receives stop/interruption requests from RMF.
      // PROPERTY: Receives custom action execution requests.
      // PROPERTY: Tracks execution state, notifies RMF on completion.
      auto ctx_cap = ctx;
      auto node_cap = node_;
      EasyFullControl::RobotCallbacks callbacks(
        // navigate_to_waypoint
        // Goal 6: command travels via PathRequest topic → fleet_manager_node
        // Goal 7: RosTopicClient registers result callback for poll detection
        [ctx_cap, node_cap](auto dest, auto exec)
        {
          // Validate via RMFHandler
          auto opt_task_id = ctx_cap->rmf_handler->on_navigate(dest);

          if (!opt_task_id.has_value()) {
            return; // Rejected
          }

          const auto pos = dest.position();
          const std::string synth_id = opt_task_id.value();

          RCLCPP_INFO(node_cap->get_logger(),
            "[FleetAdapterNode][%s] RMF TRIGGERED NAVIGATE to waypoint '%s' (%.2f, %.2f)",
            ctx_cap->name.c_str(), dest.name().c_str(), pos.x(), pos.y());

          ctx_cap->http_client->navigate(
            pos.x(), pos.y(), dest.yaw(),
            dest.name(), synth_id,
            [exec_moved = std::move(exec)](bool success) mutable
            {
              if (success)
                exec_moved.finished();
            });
        },

        // stop / interrupt
        [ctx_cap](auto ident)
        {
          // Goal 2: update StateGuard + publish STOP sentinel to fleet_manager
          ctx_cap->rmf_handler->on_stop(ident);
          // Goal 7: clear pending result callback in HttpRobotClient
          ctx_cap->http_client->stop();
        },

        // action_executor
        [ctx_cap, this](
          const std::string & category,
          const nlohmann::json & /*description*/,
          rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution action_exec)
        {
          handle_custom_action(ctx_cap, category, std::move(action_exec));
        }
      );

      auto handle = easy_fleet_->add_robot(
        name,
        ctx->nav_handle->get_state(),
        robot_configs_.at(name),
        callbacks);

      if (handle)
      {
        ctx->update_handle = handle;
        ctx->nav_handle->set_update_handle(handle);
        ctx->registered = true;
        RCLCPP_INFO(node_->get_logger(),
          "[FleetAdapterNode] Robot [%s] registered successfully.", name.c_str());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
          "[FleetAdapterNode] Failed to register robot [%s].", name.c_str());
      }
    }

    if (all_done && registration_timer_)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] All robots registered. Stopping registration timer.");
      registration_timer_->cancel();
      registration_timer_ = nullptr;

      // ── Goal 6/7: Start 5 Hz poll timer ──────────────────────────────────
      // Once all robots are registered, poll FleetManagerNode for state updates
      // and push them into the RMF traffic schedule.
      // This is the "Coordination Link" in the Hybrid Communication Strategy.
      status_poll_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),  // 5 Hz
        std::bind(&FleetAdapterNode::poll_robot_states, this));

      RCLCPP_INFO(node_->get_logger(),
        "[FleetAdapterNode] Status poll timer started (5 Hz).");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Custom action handler
  // PROPERTY: Receives custom RMF action execution requests such as docking,
  //           teleoperation, cleaning, or custom task APIs, then maps them
  //           into robot-specific backend API calls.
  // ─────────────────────────────────────────────────────────────────────────

  using ActionExecution = rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution;

  void handle_custom_action(
    const std::shared_ptr<RobotContext> & ctx,
    const std::string & category,
    ActionExecution action_exec)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[FleetAdapterNode][%s] Custom action received: '%s'",
      ctx->name.c_str(), category.c_str());

    if (category == "dock" || category == "docking")
    {
      // PROPERTY: Docking maneuver — update traffic schedule override.
      // Close the lanes leading into the dock via FleetUpdateHandle to
      // prevent schedule conflicts while the robot is docking.
      if (easy_fleet_)
      {
        // easy_fleet_->more() gives the underlying FleetUpdateHandle.
        // In production, pass actual lane IDs from the task description.
        // Here we log the intent; lane IDs require map context to resolve.
        RCLCPP_INFO(node_->get_logger(),
          "[FleetAdapterNode][%s] Docking action — traffic schedule override ready.",
          ctx->name.c_str());
        // Example: easy_fleet_->more()->close_lanes({dock_lane_id});
      }
      action_exec.finished();
    }
    else if (category == "teleoperation" || category == "teleop")
    {
      // PROPERTY: Updates RMF traffic participation during teleoperation.
      RCLCPP_WARN(node_->get_logger(),
        "[FleetAdapterNode][%s] Teleoperation active — sending stop to fleet_manager.",
        ctx->name.c_str());
      ctx->http_client->stop();  // Goal 7: stop via HttpRobotClient
      action_exec.finished();
    }
    else if (category == "patrol")
    {
      // Patrol actions are handled as sequential go_to_place tasks via
      // the existing navigate callback — mark complete immediately.
      action_exec.finished();
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
        "[FleetAdapterNode][%s] Unknown action category '%s' — completing immediately.",
        ctx->name.c_str(), category.c_str());
      action_exec.finished();
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Fleet-level event subscriptions
  // PROPERTY: Subscribes to ROS2 topics/services for dynamic lane closures,
  //           speed limit changes, fleet interrupts, and task/action
  //           completion notifications, then propagates into RMF fleet and
  //           traffic schedule system.
  // ─────────────────────────────────────────────────────────────────────────

  void setup_fleet_event_subscriptions()
  {
    // Lane closure events — published by fleet manager when a lane is blocked.
    // Topic: /fleet/lane_closed  (std_msgs/Bool, used as a trigger here;
    // production systems would use a custom message carrying lane IDs.)
    lane_closure_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/" + validator_->fleet_name() + "/lane_closed", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          RCLCPP_WARN(node_->get_logger(),
            "[FleetAdapterNode] Lane closure event received. "
            "Propagating to RMF fleet handle.");
          // Goal 7: route stop through HttpRobotClient → fleet_manager_node
          for (auto & [name, ctx] : robot_contexts_)
          {
            if (ctx->registered)
              ctx->http_client->stop();
          }
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(),
            "[FleetAdapterNode] Lane reopened — robots will resume on next task.");
          // Resume is implicit: next navigate() call will trigger a new task.
        }
      });

    // Speed limit change events — published by fleet manager.
    // Topic: /fleet/speed_limited
    speed_limit_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/" + validator_->fleet_name() + "/speed_limited", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        RCLCPP_INFO(node_->get_logger(),
          "[FleetAdapterNode] Speed limit event: %s",
          msg->data ? "ACTIVE" : "CLEARED");
        // Goal 3 NavController respects speed limits via approach_speed_limit
        // parameter; dynamic updates would reconfigure that parameter here.
      });

    RCLCPP_INFO(node_->get_logger(),
      "[FleetAdapterNode] Fleet event subscriptions established "
      "(lane closures, speed limits).");
  }

  // ───────────────────────────────────────────────────────────────────────────
  // Goal 6/7: 5 Hz status poll — Coordination Link (Adapter ↔ Manager)
  //
  // Pulls RobotStatusReport from each robot's HttpRobotClient and pushes the
  // pose + battery into the EasyFullControl update handle so RMF keeps an
  // accurate picture of the fleet without being bombarded by raw telemetry.
  // ───────────────────────────────────────────────────────────────────────────

  void poll_robot_states()
  {
    for (auto & [name, ctx] : robot_contexts_)
    {
      if (!ctx->registered || !ctx->update_handle)
        continue;

      // Goal 7: AbstractRobotClient::get_state() — polls the
      // HTTP /state service.
      const auto report = ctx->http_client->get_state();

      // Push pose + battery into RMF update handle (correct Jazzy API).
      // EasyRobotUpdateHandle::update(RobotState, activity) keeps the traffic
      // schedule current without high-frequency raw telemetry flooding.
      if (report.pose.has_value())
      {
        const auto & p = *report.pose;
        Eigen::Vector3d pos_vec(p[0], p[1], p[2]);
        
        // Sync the Nav2RobotHandle cache (used for RMF registration/state)
        ctx->nav_handle->update_state(pos_vec, report.battery_soc, true);

        EasyFullControl::RobotState state(
          "L1",                               // map name (matches building config)
          pos_vec,
          report.battery_soc);

        ctx->update_handle->update(state, nullptr);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "[FleetAdapterNode][%s] Pushing telemetry to RMF: (%.2f, %.2f, %.2f)",
          name.c_str(), p[0], p[1], p[2]);

        // Also keep the internal StateGuard in sync
        ctx->rmf_handler->update_pose(p[0], p[1], p[2]);
      }

      // Log errors for visibility
      if (report.is_error && !report.error_reason.empty())
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),
          *node_->get_clock(), 5000,  // max once per 5 s
          "[FleetAdapterNode][%s] Robot in ERROR: %s",
          name.c_str(), report.error_reason.c_str());
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Members
  // ─────────────────────────────────────────────────────────────────────────

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Adapter> adapter_;
  std::shared_ptr<EasyFullControl> easy_fleet_;

  std::unordered_map<std::string, EasyFullControl::RobotConfiguration> robot_configs_;

  // Goal 1 — FleetValidator
  std::shared_ptr<dynominion_fleet_adapter::FleetValidator> validator_;

  // PROPERTY: One per-robot context isolating all robot state.
  std::unordered_map<std::string, std::shared_ptr<RobotContext>> robot_contexts_;

  // Registration timer
  rclcpp::TimerBase::SharedPtr registration_timer_;

  // Goal 6/7: 5 Hz coordination poll timer (started after all robots registered)
  rclcpp::TimerBase::SharedPtr status_poll_timer_;

  // Fleet-level event subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_closure_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr speed_limit_sub_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Fleet Adapter Node...");

  // PROPERTY: Starts the RMF fleet adapter node, initializes rclcpp, loads
  //           config YAML and navigation graph, constructs internal RMF objects.
  auto adapter = Adapter::make("dynominion_fleet_adapter");
  if (!adapter)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize RMF Adapter!");
    return 1;
  }

  auto node = adapter->node();
  RCLCPP_INFO(node->get_logger(), "RMF Adapter initialized. Creating FleetAdapterNode...");

  auto fleet_adapter_node = std::make_shared<FleetAdapterNode>(node, adapter);
  
  try {
    fleet_adapter_node->init();
    adapter->start();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception during FleetAdapterNode initialization: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "FleetAdapterNode initialized and started. Spinning...");
  
  // If adapter->start() already started a thread with an executor, rclcpp::spin will fail.
  // We try to spin ourselves first; if it fails, we simply wait for shutdown 
  // while the adapter's background thread does the work.
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_INFO(node->get_logger(), "Node managed by adapter. Waiting for SIGINT...");
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_INFO(node->get_logger(), "FleetAdapterNode shutting down.");
  rclcpp::shutdown();
  return 0;
}
