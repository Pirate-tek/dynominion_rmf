#include <dynominion_fleet_adapter/NavController.hpp>

#include <cmath>
#include <algorithm>

namespace dynominion_fleet_adapter
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

NavController::NavController(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<RMFHandler> & rmf_handler,
  const std::shared_ptr<Nav2Handler> & nav2_handler,
  const std::string & robot_name,
  const std::string & fleet_name)
: node_(node),
  rmf_handler_(rmf_handler),
  nav2_handler_(nav2_handler),
  robot_name_(robot_name),
  fleet_name_(fleet_name)
{
  // ── Load navigation parameters ───────────────────────────────────────────
  // These are declared in fleet_node_params.yaml under navigation.*
  // Checked with has_parameter because multiple NavController instances share the same node.
  if (!node_->has_parameter("navigation.heading_error_threshold"))
    node_->declare_parameter("navigation.heading_error_threshold", 0.35);
  
  if (!node_->has_parameter("navigation.approach_radius"))
    node_->declare_parameter("navigation.approach_radius",         0.5);
  
  if (!node_->has_parameter("navigation.approach_speed_limit"))
    node_->declare_parameter("navigation.approach_speed_limit",    0.2);
  
  if (!node_->has_parameter("navigation.arrival_threshold"))
    node_->declare_parameter("navigation.arrival_threshold",       0.15);
  
  if (!node_->has_parameter("navigation.obstacle_topic_suffix"))
  {
    node_->declare_parameter("navigation.obstacle_topic_suffix",
      std::string("obstacle_detected"));
  }

  heading_error_threshold_ =
    node_->get_parameter("navigation.heading_error_threshold").as_double();
  approach_radius_ =
    node_->get_parameter("navigation.approach_radius").as_double();
  approach_speed_limit_ =
    node_->get_parameter("navigation.approach_speed_limit").as_double();
  arrival_threshold_ =
    node_->get_parameter("navigation.arrival_threshold").as_double();

  const std::string obstacle_suffix =
    node_->get_parameter("navigation.obstacle_topic_suffix").as_string();

  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Parameters: heading_err=%.2f, approach_r=%.2f, "
    "approach_v=%.2f, arrival=%.2f",
    robot_name_.c_str(),
    heading_error_threshold_, approach_radius_,
    approach_speed_limit_, arrival_threshold_);

  // ── cmd_vel publisher ────────────────────────────────────────────────────
  // Used for pre-rotation and approach speed commands.
  // Topic: /<robot_name>/cmd_vel
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/" + robot_name_ + "/cmd_vel", 10);

  // ── Obstacle subscriber ──────────────────────────────────────────────────
  // Topic: /<robot_name>/<obstacle_suffix>  (std_msgs/Bool)
  const std::string obs_topic = "/" + robot_name_ + "/" + obstacle_suffix;
  obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    obs_topic, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg)
    {
      bool prev = obstacle_detected_.load();
      obstacle_detected_.store(msg->data);
      if (msg->data && !prev)
      {
        RCLCPP_WARN(node_->get_logger(),
          "[NavController][%s] OBSTACLE DETECTED — halting translation and rotation.",
          robot_name_.c_str());
        publish_cmd_vel(0.0, 0.0);
      }
      else if (!msg->data && prev)
      {
        RCLCPP_INFO(node_->get_logger(),
          "[NavController][%s] Obstacle cleared — resuming.",
          robot_name_.c_str());
      }
    });

  // ── 50 Hz control loop ───────────────────────────────────────────────────
  control_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() { control_tick(); });

  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Initialised. Obstacle topic: '%s'.",
    robot_name_.c_str(), obs_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Public interface
// ─────────────────────────────────────────────────────────────────────────────

void NavController::navigate(double x, double y, double yaw, const std::string& task_id)
{
  std::lock_guard<std::mutex> lk(task_mtx_);

  // ── Start new task ───────────────────────────────────────────────────────
  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] New navigate request (task='%s'): (%.2f, %.2f, %.2f).",
    robot_name_.c_str(), task_id.c_str(), x, y, yaw);

  // Abort any in-flight task before accepting the new one
  if (phase_ != Phase::IDLE && phase_ != Phase::COMPLETE && phase_ != Phase::CANCELLED)
  {
    RCLCPP_WARN(node_->get_logger(),
      "[NavController][%s] Preempting in-flight task '%s' for new task '%s'.",
      robot_name_.c_str(), current_task_id_.c_str(), task_id.c_str());
    nav2_handler_->cancel_goal();
  }

  target_x_ = x;
  target_y_ = y;
  target_yaw_ = yaw;
  target_has_yaw_ = true;

  current_task_id_    = task_id;
  nav2_result_.reset();
  cancel_requested_.store(false);
  phase_              = Phase::IDLE;

  // Get initial pose from StateGuard
  const auto snap = rmf_handler_->get_state_snapshot();
  if (snap.pose.has_value())
  {
    pose_x_    = (*snap.pose)[0];
    pose_y_    = (*snap.pose)[1];
    pose_yaw_  = (*snap.pose)[2];
    pose_valid_ = true;
  }

  // Begin execution
  if (on_task_started_.has_value())
    (*on_task_started_)(current_task_id_);

  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Starting navigation sequence for task '%s'. Pose valid: %s",
    robot_name_.c_str(), task_id.c_str(), pose_valid_ ? "YES" : "NO");

  dispatch_next_waypoint();
}

void NavController::request_pause()
{
  paused_.store(true);
  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Pause requested — translation suspended.", robot_name_.c_str());
  publish_cmd_vel(0.0, 0.0);
}

void NavController::request_resume()
{
  paused_.store(false);
  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Resume requested.", robot_name_.c_str());
}

void NavController::cancel_all()
{
  cancel_requested_.store(true);
  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Cancel requested — flushing queue.", robot_name_.c_str());
  publish_cmd_vel(0.0, 0.0);

  // Async Nav2 cancel — non-blocking
  nav2_handler_->cancel_goal();
}


// ─────────────────────────────────────────────────────────────────────────────
// Waypoint dispatch
// ─────────────────────────────────────────────────────────────────────────────

void NavController::dispatch_next_waypoint()
{
  // Must be called with task_mtx_ held.

  RCLCPP_INFO(node_->get_logger(),
    "[NavController][%s] Dispatching target: "
    "(%.2f, %.2f, yaw=%.2f).",
    robot_name_.c_str(),
    target_x_, target_y_, target_yaw_);

  nav2_result_.reset();

  // Determine initial phase based on heading error
  if (pose_valid_)
  {
    double heading_err = std::abs(compute_heading_error(target_x_, target_y_));
    if (heading_err > heading_error_threshold_)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[NavController][%s] Heading error %.2f rad > threshold %.2f — pre-rotating.",
        robot_name_.c_str(), heading_err, heading_error_threshold_);
      phase_ = Phase::PRE_ROTATION;
    }
    else
    {
      phase_ = Phase::TRANSLATING;
    }
  }
  else
  {
    // No pose yet — skip pre-rotation and let Nav2 handle it
    phase_ = Phase::TRANSLATING;
  }

  // Send goal to Nav2Handler
  nav2_handler_->send_goal(
    target_x_, target_y_, target_yaw_,
    [this](Nav2Handler::GoalResult result)
    {
      std::lock_guard<std::mutex> lk(task_mtx_);
      on_nav2_result(result);
    },
    [this](double x, double y, double yaw)
    {
      on_nav2_pose(x, y, yaw);
    },
    [this](bool accepted)
    {
      on_nav2_accepted(accepted);
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Nav2 callbacks
// ─────────────────────────────────────────────────────────────────────────────

void NavController::on_nav2_result(Nav2Handler::GoalResult result)
{
  // Called with task_mtx_ held (from dispatch lambda)
  nav2_result_ = result;

  if (result == Nav2Handler::GoalResult::CANCELLED)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[NavController][%s] Nav2 goal cancelled — aborting task.", robot_name_.c_str());
    abort_task();
  }
  else if (result == Nav2Handler::GoalResult::ABORTED)
  {
    RCLCPP_WARN(node_->get_logger(),
      "[NavController][%s] Nav2 goal aborted — marking task failed.", robot_name_.c_str());
    abort_task();
  }
  // SUCCEEDED is handled in control_tick via WAITING_NAV2 → advance
}

void NavController::on_nav2_pose(double x, double y, double yaw)
{
  // Called from Nav2 feedback — NO task_mtx_ here (avoid lock inversion)
  // Update pose atomically via RMFHandler's StateGuard (thread-safe)
  rmf_handler_->update_pose(x, y, yaw);

  // Cache locally for control tick computations (task_mtx_ protects these)
  std::lock_guard<std::mutex> lk(task_mtx_);
  pose_x_    = x;
  pose_y_    = y;
  pose_yaw_  = yaw;
  pose_valid_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 50 Hz control tick
// ─────────────────────────────────────────────────────────────────────────────

void NavController::control_tick()
{
  std::lock_guard<std::mutex> lk(task_mtx_);

  // Nothing to do when idle
  if (phase_ == Phase::IDLE)
    return;

  // ── Overlay rule 1: Cancel ────────────────────────────────────────────────
  if (cancel_requested_.load())
  {
    if (phase_ != Phase::CANCELLED)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[NavController][%s] Cancel overlay active.", robot_name_.c_str());
      abort_task();
    }
    return;
  }

  // ── Terminal phases ───────────────────────────────────────────────────────
  if (phase_ == Phase::COMPLETE)
  {
    complete_task();
    return;
  }
  if (phase_ == Phase::CANCELLED)
    return;

  // ── Overlay rule 2: Obstacle — halt both axes ─────────────────────────────
  if (obstacle_detected_.load())
  {
    publish_cmd_vel(0.0, 0.0);
    return;
  }

  // ── Overlay rule 3: Pause — suspend translation only ─────────────────────
  if (paused_.load() && phase_ != Phase::PRE_ROTATION)
  {
    publish_cmd_vel(0.0, 0.0);
    return;
  }

  if (!pose_valid_)
    return;  // Wait for first pose

  double dist         = compute_distance(target_x_, target_y_);
  double heading_err  = compute_heading_error(target_x_, target_y_);

  // ── PRE_ROTATION phase ────────────────────────────────────────────────────
  if (phase_ == Phase::PRE_ROTATION)
  {
    // Issue in-place rotation until heading error is within tolerance.
    // Nav2Handler has already been sent the goal — it will handle actual nav.
    // The cmd_vel here is for fine alignment before Nav2 takes control.
    constexpr double kAngularGain = 1.2;
    double angular_z = std::clamp(
      kAngularGain * heading_err, -1.5, 1.5);

    if (std::abs(heading_err) <= heading_error_threshold_ * 0.5)
    {
      // Aligned — transition to TRANSLATING
      RCLCPP_INFO(node_->get_logger(),
        "[NavController][%s] Pre-rotation complete (err=%.3f rad). Beginning translation.",
        robot_name_.c_str(), heading_err);
      publish_cmd_vel(0.0, 0.0);
      phase_ = Phase::TRANSLATING;
    }
    else
    {
      publish_cmd_vel(0.0, angular_z);
    }
    return;
  }

  // ── WAITING_NAV2 phase — distance reached, awaiting Nav2 confirmation ─────
  if (phase_ == Phase::WAITING_NAV2)
  {
    if (nav2_result_.has_value() && *nav2_result_ == Nav2Handler::GoalResult::SUCCEEDED)
    {
      // Nav2 confirmed success — advance to next waypoint
      nav2_result_.reset();
      if (target_has_yaw_)
      {
        RCLCPP_INFO(node_->get_logger(),
          "[NavController][%s] Target reached — aligning to final yaw %.2f.",
          robot_name_.c_str(), target_yaw_);
        phase_ = Phase::FINAL_ROTATION;
      }
      else
      {
        phase_ = Phase::COMPLETE;
      }
    }
    // else: still waiting — do nothing
    return;
  }

  // ── FINAL_ROTATION phase ──────────────────────────────────────────────────
  if (phase_ == Phase::FINAL_ROTATION)
  {
    double yaw_err = target_yaw_ - pose_yaw_;
    // Normalize to [-pi, pi]
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    constexpr double kAngularGain = 0.8;
    constexpr double kYawTol      = 0.12;  // rad

    if (std::abs(yaw_err) <= kYawTol)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[NavController][%s] Final yaw alignment complete (err=%.3f).",
        robot_name_.c_str(), yaw_err);
      publish_cmd_vel(0.0, 0.0);
      phase_ = Phase::COMPLETE;
    }
    else
    {
      double angular_z = std::clamp(kAngularGain * yaw_err, -1.2, 1.2);
      publish_cmd_vel(0.0, angular_z);
    }
    return;
  }

  // ── TRANSLATING / APPROACH phases ─────────────────────────────────────────

  // Transition to APPROACH zone
  if (phase_ == Phase::TRANSLATING && dist <= approach_radius_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[NavController][%s] Entering approach zone (dist=%.2f m).",
      robot_name_.c_str(), dist);
    phase_ = Phase::APPROACH;
  }

  // Check arrival threshold — enter WAITING_NAV2
  if (dist <= arrival_threshold_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[NavController][%s] Distance threshold reached (dist=%.3f m). "
      "Awaiting Nav2 confirmation.",
      robot_name_.c_str(), dist);
    publish_cmd_vel(0.0, 0.0);
    phase_ = Phase::WAITING_NAV2;
    return;
  }

  // Navigation is handled by Nav2 — cmd_vel here provides overlay corrections
  // (speed scaling in approach zone). In practice, Nav2 sends its own cmd_vel;
  // the controller only intervenes for pre-rotation and final alignment.
  // The approach speed limit is enforced by gating the Nav2 goal's speed in
  // the future; for now we publish zero override to avoid conflicting with Nav2.
  (void)approach_speed_limit_;
}

void NavController::on_nav2_accepted(bool accepted)
{
  std::lock_guard<std::mutex> lk(task_mtx_);
  if (accepted)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[NavController][%s] Nav2 goal accepted for task '%s'.",
      robot_name_.c_str(), current_task_id_.c_str());
    if (on_task_accepted_.has_value())
      (*on_task_accepted_)(current_task_id_);
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[NavController][%s] Nav2 goal rejected/unavailable for task '%s'.",
      robot_name_.c_str(), current_task_id_.c_str());
  }
}

void NavController::update_pose(double x, double y, double yaw)
{
  std::lock_guard<std::mutex> lk(task_mtx_);
  pose_x_    = x;
  pose_y_    = y;
  pose_yaw_  = yaw;
  pose_valid_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Task completion / abort
// ─────────────────────────────────────────────────────────────────────────────

void NavController::complete_task()
{
  // Must be called with task_mtx_ held.
  phase_ = Phase::IDLE;

  if (on_task_finished_.has_value())
    (*on_task_finished_)(true, "");

  current_task_id_.clear();
  nav2_result_.reset();
}

void NavController::abort_task()
{
  // Must be called with task_mtx_ held (or from cancel_all which holds no task lock).
  phase_ = Phase::CANCELLED;
  publish_cmd_vel(0.0, 0.0);

  RCLCPP_WARN(node_->get_logger(),
    "[NavController][%s] Task '%s' ABORTED.",
    robot_name_.c_str(), current_task_id_.c_str());

  current_task_id_.clear();
  nav2_result_.reset();

  // Reset to IDLE after a brief delay (next tick)
  phase_ = Phase::IDLE;
  cancel_requested_.store(false);

  // ── Goal 5: notify state machine of failure ──────────────────────────────
  if (on_task_finished_.has_value())
    (*on_task_finished_)(false, "task aborted");
}

// ─────────────────────────────────────────────────────────────────────────────
// Geometry helpers
// ─────────────────────────────────────────────────────────────────────────────

double NavController::compute_heading_error(double tx, double ty) const
{
  double dx      = tx - pose_x_;
  double dy      = ty - pose_y_;
  double desired = std::atan2(dy, dx);
  double err     = desired - pose_yaw_;
  // Normalize to [-pi, pi]
  while (err >  M_PI) err -= 2.0 * M_PI;
  while (err < -M_PI) err += 2.0 * M_PI;
  return err;
}

double NavController::compute_distance(double tx, double ty) const
{
  double dx = tx - pose_x_;
  double dy = ty - pose_y_;
  return std::sqrt(dx * dx + dy * dy);
}

// ─────────────────────────────────────────────────────────────────────────────
// cmd_vel publisher
// ─────────────────────────────────────────────────────────────────────────────

void NavController::publish_cmd_vel(double linear_x, double angular_z)
{
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp = node_->now();
  twist_stamped.header.frame_id = robot_name_ + "/base_footprint";
  twist_stamped.twist.linear.x  = linear_x;
  twist_stamped.twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist_stamped);
}

}  // namespace dynominion_fleet_adapter
