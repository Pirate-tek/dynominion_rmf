#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>
#include <dynominion_fleet_adapter/Nav2RobotHandle.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <Eigen/Dense>

using namespace rmf_fleet_adapter::agv;

// Helper to compute transformation between RMF and robot coordinates
// This implements the same algorithm as the 'nudged' Python library
Transformation compute_transformation(
  const std::string& level,
  const YAML::Node& coords,
  const rclcpp::Logger& logger)
{
  auto rmf_node = coords["rmf"];
  auto robot_node = coords["robot"];

  if (!rmf_node || !robot_node || rmf_node.size() < 2 || robot_node.size() < 2)
  {
    RCLCPP_WARN(logger, "Not enough points for transformation on level [%s]. Using default.", level.c_str());
    return Transformation(0.0, 1.0, Eigen::Vector2d::Zero());
  }

  int n = std::min((int)rmf_node.size(), (int)robot_node.size());
  
  double sum_x = 0, sum_y = 0;
  double sum_xp = 0, sum_yp = 0;
  
  for (int i = 0; i < n; ++i)
  {
    sum_x += rmf_node[i][0].as<double>();
    sum_y += rmf_node[i][1].as<double>();
    sum_xp += robot_node[i][0].as<double>();
    sum_yp += robot_node[i][1].as<double>();
  }
  
  double mean_x = sum_x / n;
  double mean_y = sum_y / n;
  double mean_xp = sum_xp / n;
  double mean_yp = sum_yp / n;
  
  double s_uu_vv = 0;
  double s_uup_vvp = 0;
  double s_uvp_vup = 0;
  
  for (int i = 0; i < n; ++i)
  {
    double u = rmf_node[i][0].as<double>() - mean_x;
    double v = rmf_node[i][1].as<double>() - mean_y;
    double up = robot_node[i][0].as<double>() - mean_xp;
    double vp = robot_node[i][1].as<double>() - mean_yp;
    
    s_uu_vv += u*u + v*v;
    s_uup_vvp += u*up + v*vp;
    s_uvp_vup += u*vp - v*up;
  }
  
  if (s_uu_vv < 1e-9)
  {
    RCLCPP_WARN(logger, "Points are too close to compute transformation for level [%s].", level.c_str());
    return Transformation(0.0, 1.0, Eigen::Vector2d(mean_xp - mean_x, mean_yp - mean_y));
  }
  
  double a = s_uup_vvp / s_uu_vv;
  double b = s_uvp_vup / s_uu_vv;
  
  double tx = mean_xp - (a * mean_x - b * mean_y);
  double ty = mean_yp - (b * mean_x + a * mean_y);
  
  double rotation = std::atan2(b, a);
  double scale = std::sqrt(a * a + b * b);
  Eigen::Vector2d translation(tx, ty);

  RCLCPP_INFO(logger, "Computed transformation for [%s]: rot=%.3f, scale=%.3f, trans=(%.3f, %.3f)",
    level.c_str(), rotation, scale, tx, ty);

  return Transformation(rotation, scale, translation);
}

class FleetAdapterNode
{
public:
   FleetAdapterNode(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Adapter>& adapter)
  : node_(node), adapter_(adapter)
  {}

  void init()
  {
    node_->declare_parameter("config_file", "");
    node_->declare_parameter("nav_graph_path", "");

    std::string config_file = node_->get_parameter("config_file").as_string();
    if (config_file.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Config file path is empty!");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Loading config file: %s", config_file.c_str());
    YAML::Node config;
    try
    {
      config = YAML::LoadFile(config_file);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", e.what());
      return;
    }

    // Parse RMF Fleet Info
    auto rmf_fleet_node = config["rmf_fleet"];
    if (!rmf_fleet_node)
    {
      RCLCPP_ERROR(node_->get_logger(), "Config missing 'rmf_fleet' section!");
      return;
    }

    std::string fleet_name = rmf_fleet_node["name"].as<std::string>();
    RCLCPP_INFO(node_->get_logger(), "Initializing fleet: %s", fleet_name.c_str());

    // Vehicle Traits
    auto limits_node = rmf_fleet_node["limits"];
    auto profile_node = rmf_fleet_node["profile"];
    
    auto footprint = rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
      profile_node["footprint_radius"].as<double>());
    
    auto traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
      rmf_traffic::agv::VehicleTraits::Limits{limits_node["max_linear_speed"].as<double>(), limits_node["max_linear_acceleration"].as<double>()},
      rmf_traffic::agv::VehicleTraits::Limits{limits_node["max_angular_speed"].as<double>(), limits_node["max_angular_acceleration"].as<double>()},
      rmf_traffic::Profile(footprint)
    );
    traits->set_differential(rmf_traffic::agv::VehicleTraits::Differential(Eigen::Vector2d::UnitX(), true));

    // Nav Graph
    std::string graph_path = node_->get_parameter("nav_graph_path").as_string();
    if (graph_path.empty())
    {
       RCLCPP_ERROR(node_->get_logger(), "Nav graph path is empty!");
       return;
    }
    auto graph = std::make_shared<rmf_traffic::agv::Graph>(
      parse_graph(graph_path, *traits));
    RCLCPP_INFO(node_->get_logger(), "Loaded graph with %zu waypoints", graph->num_waypoints());

    // Robot Configurations (Chargers)
    auto robots_node = rmf_fleet_node["robots"];
    if (robots_node)
    {
      for (auto it = robots_node.begin(); it != robots_node.end(); ++it)
      {
        std::string r_name = it->first.as<std::string>();
        std::string charger = it->second["charger"].as<std::string>();
        robot_configs_.emplace(r_name, EasyFullControl::RobotConfiguration({charger}));
        RCLCPP_INFO(node_->get_logger(), "Configured robot [%s] with charger [%s]", r_name.c_str(), charger.c_str());
      }
    }

    EasyFullControl::FleetConfiguration fleet_config(
      fleet_name,
      std::nullopt, // transformations (added below)
      robot_configs_,
      traits,
      graph,
      nullptr, // battery_system
      nullptr, // motion_sink
      nullptr, // ambient_sink
      nullptr, // tool_sink
      0.2,     // recharge_threshold
      0.9,     // recharge_soc
      false,   // account_for_battery_drain
      {},      // task_consideration
      {},      // action_consideration
      nullptr, // finishing_request
      false,   // skip_wait_until
      std::nullopt, // server_uri
      std::chrono::seconds(10), // min_hold_time
      std::chrono::seconds(2),  // update_interval
      true,    // publish_fleet_state
      2.0,     // max_merge_waypoint_distance
      2.0,     // max_merge_lane_distance
      0.3      // min_lane_width
    );

    // Add transformations from reference_coordinates
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

    // Add fleet to adapter
    easy_fleet_ = adapter_->add_easy_fleet(fleet_config);
    if (!easy_fleet_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to add easy fleet!");
      return;
    }

    // Initialize Robot Handles
    for (const auto& [name, cfg] : robot_configs_)
    {
      auto nav_handle = std::make_shared<Nav2RobotHandle>(name, node_);
      pending_robots_.push_back(nav_handle);
    }

    // Registration Timer
    registration_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FleetAdapterNode::check_registration, this));

    adapter_->start();
  }

private:
  void check_registration()
  {
    auto it = pending_robots_.begin();
    while (it != pending_robots_.end())
    {
      auto nav_handle = *it;
      if (nav_handle->is_ready())
      {
        std::string robot_name = nav_handle->name();
        RCLCPP_INFO(node_->get_logger(), "Robot [%s] is ready. Registering...", robot_name.c_str());

        EasyFullControl::RobotCallbacks callbacks(
          [nav_handle](auto dest, auto exec) { nav_handle->navigate(dest, std::move(exec)); },
          [nav_handle](auto ident) { nav_handle->stop(ident); },
          nullptr
        );

        auto handle = easy_fleet_->add_robot(
          robot_name,
          nav_handle->get_state(),
          robot_configs_.at(robot_name),
          callbacks
        );

        if (handle)
        {
          nav_handle->set_update_handle(handle);
          it = pending_robots_.erase(it);
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to add robot [%s]", robot_name.c_str());
          ++it;
        }
      }
      else
      {
        ++it;
      }
    }

    if (pending_robots_.empty() && registration_timer_)
    {
      registration_timer_->cancel();
      registration_timer_ = nullptr;
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Adapter> adapter_;
  std::shared_ptr<EasyFullControl> easy_fleet_;
  std::unordered_map<std::string, EasyFullControl::RobotConfiguration> robot_configs_;
  std::vector<std::shared_ptr<Nav2RobotHandle>> pending_robots_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto adapter = Adapter::make("dynominion_fleet_adapter");
  if (!adapter)
    return 1;
  
  auto node = adapter->node();
  auto fleet_adapter_node = std::make_shared<FleetAdapterNode>(node, adapter);
  fleet_adapter_node->init();
  
  // The RMF adapter handles spinning the node in its own background thread.
  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();
  return 0;
}
