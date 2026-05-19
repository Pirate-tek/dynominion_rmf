#include <dynominion_fleet_adapter/FleetManagerNode.hpp>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dynominion_fleet_adapter
{
using json = nlohmann::json;

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

  // ── Goal 1: Fleet Identity Validation ────────────────────────────────────
  // FleetValidator reads fleet_identity.* parameters declared in
  // fleet_node_params.yaml (same config used by fleet_adapter_node).
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

  // ── Build per-robot contexts ──────────────────────────────────────────────
  for (auto it = robots_node.begin(); it != robots_node.end(); ++it)
  {
    const std::string robot_name = it->first.as<std::string>();
    setup_robot(robot_name, fleet_name);
  }

  setup_http_server();

  // Start HTTP server in a separate thread
  http_thread_ = std::thread([this]() {
    RCLCPP_INFO(get_logger(), "[FleetManagerNode] Starting HTTP server on port 8080");
    http_server_.listen("0.0.0.0", 8080);
  });

  RCLCPP_INFO(get_logger(),
    "[FleetManagerNode] Initialised. Managing %zu robot(s) in fleet '%s'.",
    robots_.size(), fleet_name.c_str());
}

FleetManagerNode::~FleetManagerNode()
{
  http_server_.stop();
  if (http_thread_.joinable())
    http_thread_.join();
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

  // ── Goal 5: State Machine ─────────────────────────────────────────────────
  ctx->state_machine = std::make_shared<RobotStateMachine>(
    robot_name, get_logger());

  // Wire state machine diagnostics to ROS logging
  ctx->state_machine->set_transition_callback(
    [this, robot_name](RobotState from, RobotState to, const std::string & ctx_msg)
    {
      // Publish a diagnostic-level event for any monitoring tools.
      RCLCPP_INFO(get_logger(),
        "[FleetManagerNode][%s] State: %s → %s  (%s)",
        robot_name.c_str(),
        to_string(from), to_string(to),
        ctx_msg.c_str());
    });

  // ── Goal 2: RMFHandler (StateGuard + topic bridge) ────────────────────────
  ctx->rmf_handler = std::make_shared<RMFHandler>(
    shared_from_this(), validator_, robot_name);

  // ── Goal 4: Nav2Handler (action client) ──────────────────────────────────
  ctx->nav2_handler = std::make_shared<Nav2Handler>(
    shared_from_this(), robot_name);

  // ── Telemetry Subscriptions ──────────────────────────────────────────────
  // PROPERTY: Subscribes to Nav2/Robot telemetry to keep the state machine 
  //           and RMF schedule current.
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
      {
        ctx->nav_controller->update_pose(p.x, p.y, yaw);
      }
    });

  ctx->battery_sub = create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + robot_name + "/battery_state", 10,
    [ctx](const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
      ctx->rmf_handler->update_battery(msg->percentage);
    });

  // ── Goal 3: NavController (sequencer, 50 Hz control loop) ────────────────
  ctx->nav_controller = std::make_shared<NavController>(
    shared_from_this(), ctx->rmf_handler, ctx->nav2_handler,
    robot_name, fleet_name);

  // Connect NavController state-change hooks to the state machine.
  // The NavController calls these lambdas on its completion/abort paths.
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

  ctx->nav_controller->set_on_task_finished(
    [ctx](bool success, const std::string & reason)
    {
      if (success)
        ctx->state_machine->on_success();
      else
        ctx->state_machine->on_error(reason);
    });


  RCLCPP_INFO(get_logger(),
    "[FleetManagerNode] Robot [%s] ready.", robot_name.c_str());

  robots_.emplace(robot_name, ctx);
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP Server implementation
// ─────────────────────────────────────────────────────────────────────────────

void FleetManagerNode::setup_http_server()
{
  // GET /v1/robots/{id}/state
  http_server_.Get(R"(/v1/robots/([^/]+)/state)", [this](const httplib::Request& req, httplib::Response& res) {
    std::string robot_name = req.matches[1];
    auto it = robots_.find(robot_name);
    if (it == robots_.end()) {
      res.status = 404;
      return;
    }
    const auto & ctx = it->second;
    const auto snap = ctx->rmf_handler->get_state_snapshot();

    json j;
    j["success"] = true;
    j["state"] = to_string(ctx->state_machine->state());
    j["task_id"] = ctx->state_machine->active_task_id();
    j["error_reason"] = ctx->state_machine->last_error_reason();
    j["battery"] = snap.battery_soc;
    
    if (snap.pose.has_value()) {
      j["pose"] = {(*snap.pose)[0], (*snap.pose)[1], (*snap.pose)[2]};
    }

    res.set_content(j.dump(), "application/json");
  });

  // POST /v1/robots/{id}/navigate
  http_server_.Post(R"(/v1/robots/([^/]+)/navigate)", [this](const httplib::Request& req, httplib::Response& res) {
    std::string robot_name = req.matches[1];
    auto it = robots_.find(robot_name);
    if (it == robots_.end()) {
      res.status = 404;
      return;
    }
    
    try {
      json j = json::parse(req.body);
      double x = j.at("x").get<double>();
      double y = j.at("y").get<double>();
      double yaw = j.at("yaw").get<double>();
      std::string task_id = j.at("task_id").get<std::string>();

      RCLCPP_INFO(get_logger(),
        "[FleetManagerNode][%s] HTTP NAVIGATE RECEIVED: target=(%.2f, %.2f) task='%s'",
        robot_name.c_str(), x, y, task_id.c_str());

      it->second->nav_controller->navigate(x, y, yaw, task_id);
      
      json resp;
      resp["success"] = true;
      res.set_content(resp.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(std::string("Invalid JSON: ") + e.what(), "text/plain");
    }
  });

  // POST /v1/robots/{id}/stop
  http_server_.Post(R"(/v1/robots/([^/]+)/stop)", [this](const httplib::Request& req, httplib::Response& res) {
    std::string robot_name = req.matches[1];
    auto it = robots_.find(robot_name);
    if (it == robots_.end()) {
      res.status = 404;
      return;
    }
    it->second->nav_controller->cancel_all();
    json resp;
    resp["success"] = true;
    res.set_content(resp.dump(), "application/json");
  });

  // POST /v1/robots/{id}/recover
  http_server_.Post(R"(/v1/robots/([^/]+)/recover)", [this](const httplib::Request& req, httplib::Response& res) {
    std::string robot_name = req.matches[1];
    auto it = robots_.find(robot_name);
    if (it == robots_.end()) {
      res.status = 404;
      return;
    }
    it->second->state_machine->on_recover();
    json resp;
    resp["success"] = true;
    res.set_content(resp.dump(), "application/json");
  });
}

}  // namespace dynominion_fleet_adapter

// ─────────────────────────────────────────────────────────────────────────────
// main — fleet_manager_node executable (Goal 6, Process 2)
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "[fleet_manager_node] Starting Fleet Manager Node...");

  auto node = std::make_shared<dynominion_fleet_adapter::FleetManagerNode>();
  node->init();

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "[fleet_manager_node] Shutting down.");
  rclcpp::shutdown();
  return 0;
}
