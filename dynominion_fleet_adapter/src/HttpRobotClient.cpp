#include <dynominion_fleet_adapter/HttpRobotClient.hpp>
#include <dynominion_fleet_adapter/thirdparty/httplib.h>
#include <dynominion_fleet_adapter/thirdparty/json.hpp>

using json = nlohmann::json;

namespace dynominion_fleet_adapter
{

HttpRobotClient::HttpRobotClient(
  rclcpp::Node::SharedPtr node,
  const std::string & robot_name,
  const std::string & host,
  int port)
: node_(std::move(node)),
  robot_name_(robot_name)
{
  http_client_ = std::make_unique<httplib::Client>(host, port);
  http_client_->set_connection_timeout(1, 0); // 1s
  http_client_->set_read_timeout(1, 0); // 1s

  RCLCPP_INFO(node_->get_logger(),
    "[HttpRobotClient][%s] Created. Connecting to http://%s:%d",
    robot_name_.c_str(), host.c_str(), port);
}

HttpRobotClient::~HttpRobotClient() = default;

// ─────────────────────────────────────────────────────────────────────────────
// navigate
// ─────────────────────────────────────────────────────────────────────────────

bool HttpRobotClient::navigate(
  double x, double y, double yaw,
  const std::string & /*waypoint_name*/,
  const std::string & task_id,
  NavigationResultCallback on_result)
{
  json payload = {
    {"x", x},
    {"y", y},
    {"yaw", yaw},
    {"task_id", task_id}
  };

  std::string endpoint = "/v1/robots/" + robot_name_ + "/navigate";
  auto res = http_client_->Post(endpoint, payload.dump(), "application/json");

  if (!res || res->status != 200) {
    RCLCPP_ERROR(node_->get_logger(),
      "[HttpRobotClient][%s] Failed to send navigate command. Status: %d",
      robot_name_.c_str(), res ? res->status : -1);
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[HttpRobotClient][%s] SENDING NAVIGATE: target=(%.2f, %.2f, yaw=%.2f) task='%s' endpoint='%s'",
    robot_name_.c_str(), x, y, yaw, task_id.c_str(), endpoint.c_str());

  pending_task_id_   = task_id;
  task_started_      = false;
  pending_result_cb_ = std::move(on_result);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// stop
// ─────────────────────────────────────────────────────────────────────────────

void HttpRobotClient::stop()
{
  std::string endpoint = "/v1/robots/" + robot_name_ + "/stop";
  auto res = http_client_->Post(endpoint);

  if (!res || res->status != 200) {
    RCLCPP_ERROR(node_->get_logger(),
      "[HttpRobotClient][%s] Failed to send stop command. Status: %d",
      robot_name_.c_str(), res ? res->status : -1);
  } else {
    RCLCPP_INFO(node_->get_logger(),
      "[HttpRobotClient][%s] stop() — command accepted.", robot_name_.c_str());
  }

  pending_task_id_.clear();
  pending_result_cb_ = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// get_state
// ─────────────────────────────────────────────────────────────────────────────

RobotStatusReport HttpRobotClient::get_state()
{
  RobotStatusReport report;
  std::string endpoint = "/v1/robots/" + robot_name_ + "/state";
  
  auto res = http_client_->Get(endpoint);

  if (!res || res->status != 200)
  {
    RCLCPP_DEBUG(node_->get_logger(),
      "[HttpRobotClient][%s] get_state failed or timed out.", robot_name_.c_str());
    return report;
  }

  try {
    json j = json::parse(res->body);
    
    if (j.contains("pose") && j["pose"].is_array() && j["pose"].size() == 3) {
      report.pose = {
        j["pose"][0].get<double>(),
        j["pose"][1].get<double>(),
        j["pose"][2].get<double>()
      };
    }

    if (j.contains("battery")) {
      report.battery_soc = j["battery"].get<double>();
    }

    std::string state_str = "IDLE";
    if (j.contains("state")) {
      state_str = j["state"].get<std::string>();
    }

    report.is_navigating = (state_str == "NAVIGATING");
    report.is_error      = (state_str == "ERROR");

    if (j.contains("error_reason")) {
      report.error_reason = j["error_reason"].get<std::string>();
    }

    std::string current_task_id = "";
    if (j.contains("task_id")) {
      current_task_id = j["task_id"].get<std::string>();
    }

    // Detect task completion
    if (!pending_task_id_.empty() && pending_result_cb_)
    {
      if (current_task_id == pending_task_id_)
      {
        task_started_ = true;
      }

      if (task_started_)
      {
        if (state_str == "IDLE" && current_task_id != pending_task_id_)
        {
          RCLCPP_INFO(node_->get_logger(),
            "[HttpRobotClient][%s] Task '%s' detected COMPLETE (poll).",
            robot_name_.c_str(), pending_task_id_.c_str());
          auto cb = std::move(pending_result_cb_);
          pending_task_id_.clear();
          task_started_ = false;
          cb(true);
        }
        else if (state_str == "ERROR")
        {
          RCLCPP_WARN(node_->get_logger(),
            "[HttpRobotClient][%s] Task '%s' detected ERROR: %s",
            robot_name_.c_str(), pending_task_id_.c_str(), report.error_reason.c_str());
          auto cb = std::move(pending_result_cb_);
          pending_task_id_.clear();
          task_started_ = false;
          cb(false);
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(),
      "[HttpRobotClient][%s] Error parsing state JSON: %s",
      robot_name_.c_str(), e.what());
  }

  return report;
}

// ─────────────────────────────────────────────────────────────────────────────
// recover
// ─────────────────────────────────────────────────────────────────────────────

void HttpRobotClient::recover()
{
  std::string endpoint = "/v1/robots/" + robot_name_ + "/recover";
  auto res = http_client_->Post(endpoint);

  if (!res || res->status != 200) {
    RCLCPP_ERROR(node_->get_logger(),
      "[HttpRobotClient][%s] Failed to send recover command. Status: %d",
      robot_name_.c_str(), res ? res->status : -1);
  } else {
    RCLCPP_INFO(node_->get_logger(),
      "[HttpRobotClient][%s] recover() requested via API.", robot_name_.c_str());
  }
}

}  // namespace dynominion_fleet_adapter
