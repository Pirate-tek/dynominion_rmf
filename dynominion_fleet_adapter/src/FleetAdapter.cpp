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

class FleetAdapterNode
{
public:
  FleetAdapterNode(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<rmf_fleet_adapter::agv::Adapter>& adapter)
  : node_(node), adapter_(adapter)
  {}

  void init()
  {
    node_->declare_parameter("config_file", "");
    node_->declare_parameter("nav_graph_path", "");

    std::string config_file = node_->get_parameter("config_file").as_string();
    if (config_file.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Config file not provided!");
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

    if (!config["fleet_manager"])
    {
      RCLCPP_ERROR(node_->get_logger(), "Config file missing 'fleet_manager' section!");
      return;
    }

    fleet_name_ = config["fleet_manager"]["fleet_name"].as<std::string>();
    RCLCPP_INFO(node_->get_logger(), "Initializing fleet: %s", fleet_name_.c_str());
    
    // Load Nav Graph
    std::string graph_path = node_->get_parameter("nav_graph_path").as_string();
    if (graph_path.empty() && config["fleet_manager"]["nav_graph_path"])
    {
      graph_path = config["fleet_manager"]["nav_graph_path"].as<std::string>();
    }
    
    if (graph_path.empty())
    {
       RCLCPP_ERROR(node_->get_logger(), "Nav graph path is empty!");
       return;
    }
    RCLCPP_INFO(node_->get_logger(), "Loading nav graph: %s", graph_path.c_str());

    // Vehicle Traits
    auto profile_node = config["fleet_manager"]["robot_profile"];
    if (!profile_node)
    {
       RCLCPP_ERROR(node_->get_logger(), "Missing 'robot_profile' in config!");
       return;
    }

    auto footprint = rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
      profile_node["footprint_radius"].as<double>());
    
    auto traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
      rmf_traffic::agv::VehicleTraits::Limits{profile_node["max_linear_speed"].as<double>(), profile_node["max_linear_acceleration"].as<double>()},
      rmf_traffic::agv::VehicleTraits::Limits{profile_node["max_angular_speed"].as<double>(), profile_node["max_angular_acceleration"].as<double>()},
      rmf_traffic::Profile(footprint)
    );
    traits->set_differential(rmf_traffic::agv::VehicleTraits::Differential(Eigen::Vector2d::UnitX(), profile_node["can_reverse"].as<bool>()));

    // Read graph
    auto graph = std::make_shared<rmf_traffic::agv::Graph>(
      rmf_fleet_adapter::agv::parse_graph(graph_path, *traits));
    RCLCPP_INFO(node_->get_logger(), "Successfuly loaded graph with %zu waypoints", graph->num_waypoints());

    // Fleet Configuration
    std::unordered_map<std::string, rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration> robot_configs;
    auto robots = config["fleet_manager"]["robots"];
    for (auto it = robots.begin(); it != robots.end(); ++it)
    {
      std::string robot_name = it->first.as<std::string>();
      std::string charger_name = it->second["charger"].as<std::string>();
      robot_configs.emplace(robot_name, rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration({charger_name}));
    }

    rmf_fleet_adapter::agv::EasyFullControl::FleetConfiguration fleet_config(
      fleet_name_,
      std::nullopt,
      robot_configs,
      traits,
      graph,
      nullptr, // battery system
      nullptr, // motion sink
      nullptr, // ambient sink
      nullptr, // tool sink
      0.2,     // recharge threshold
      0.9,     // recharge soc
      false,   // account for drain
      {},      // task consideration
      {}       // action consideration
    );

    // EasyFullControl registration
    RCLCPP_INFO(node_->get_logger(), "Adding easy fleet...");
    easy_fleet_ = adapter_->add_easy_fleet(fleet_config);
    if (!easy_fleet_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to add easy fleet!");
      return;
    }

    // Register Robots
    RCLCPP_INFO(node_->get_logger(), "Registering %zu robots...", robots.size());
    for (auto it = robots.begin(); it != robots.end(); ++it)
    {
      std::string robot_name = it->first.as<std::string>();
      RCLCPP_INFO(node_->get_logger(), "Registering robot: %s", robot_name.c_str());
      
      auto nav_handle = std::make_shared<Nav2RobotHandle>(
        robot_name, node_);

      rmf_fleet_adapter::agv::EasyFullControl::RobotCallbacks robot_callbacks(
        [nav_handle](auto dest, auto exec) { nav_handle->navigate(dest, std::move(exec)); },
        [nav_handle](auto ident) { nav_handle->stop(ident); },
        nullptr
      );

      auto robot_update_handle = easy_fleet_->add_robot(
        robot_name,
        rmf_fleet_adapter::agv::EasyFullControl::RobotState("L1", Eigen::Vector3d(0.0, 0.0, 0.0), 1.0),
        robot_configs.at(robot_name),
        robot_callbacks
      );

      if (!robot_update_handle)
      {
         RCLCPP_ERROR(node_->get_logger(), "Failed to add robot: %s", robot_name.c_str());
         continue;
      }

      nav_handle->set_update_handle(robot_update_handle);
      robot_handles_.push_back(nav_handle);
    }

    RCLCPP_INFO(node_->get_logger(), "Starting RMF adapter...");
    adapter_->start();
  }

private:
  std::string fleet_name_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rmf_fleet_adapter::agv::Adapter> adapter_;
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> easy_fleet_;
  std::vector<std::shared_ptr<Nav2RobotHandle>> robot_handles_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto adapter = rmf_fleet_adapter::agv::Adapter::init_and_make("dynominion_fleet_adapter");
  if (!adapter)
  {
    std::cerr << "Failed to initialize RMF adapter" << std::endl;
    return 1;
  }
  
  auto node = adapter->node();
  auto fleet_adapter_node = std::make_shared<FleetAdapterNode>(node, adapter);
  fleet_adapter_node->init();
  
  RCLCPP_INFO(node->get_logger(), "Node initialization complete, spinning...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
