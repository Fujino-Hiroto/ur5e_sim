// coarse_to_fine_go_to_leaf3.cpp
//
// Coarse->Fine approach to a leaf target using MoveIt2, TF and a PointCloud2 “forward ROI” monitor.
// - COARSE: move to a pre-approach position (down + back) while looking at leaf_center_frame.
// - FINE: move to leaf_target_shot (prefer Cartesian if available) while looking at leaf_center_frame.
// - VIEW: rotate in place to match leaf_target_view orientation.
//
// Key safety feature:
// - Runtime “forward obstacle” monitor using PointCloud2 in camera optical frame (+Z forward).
// - Stop & replan when an obstacle is closer than a threshold.
// - Goal proximity gating + “leaf self ignore” to avoid stopping forever due to the leaf itself.
//
// IMPORTANT (Node split to avoid executor conflict):
// - app_node: TF / PointCloud monitor / RTAB-Map params / control logic
// - moveit_node: MoveGroupInterface only
//
// We run our own executor for app_node (always spinning).
// We DO NOT add moveit_node to our executor, because MoveGroupInterface internally adds its node
// to its own executor/spin-thread. Adding the same node to two executors causes:
//   "Node '...' has already been added to an executor."
//
// Also, do NOT call rclcpp::spin_some(...) anywhere.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <chrono>
#include <thread>
#include <atomic>

// ============================================================
// Small helpers
// ============================================================
static double dist_xyz(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static std::chrono::nanoseconds sec_to_ns(double sec)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(sec));
}

static bool is_finite3(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

static double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

static double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Normalize angle to [-pi, +pi]
static double norm_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

static moveit_msgs::msg::Constraints make_wrist1_joint_constraint(
    const std::string& joint_name,
    double center_rad,
    double tol_rad)
{
  moveit_msgs::msg::Constraints c;
  moveit_msgs::msg::JointConstraint jc;

  jc.joint_name = joint_name;
  jc.position = norm_angle(center_rad);     // center in [-pi, +pi]
  jc.tolerance_above = std::fabs(tol_rad);  // symmetric tolerance
  jc.tolerance_below = std::fabs(tol_rad);
  jc.weight = 1.0;

  c.joint_constraints.push_back(jc);
  return c;
}

static moveit_msgs::msg::Constraints make_ee_orientation_constraint(
    const std::string& link_name,
    const std::string& frame_id,
    const geometry_msgs::msg::Quaternion& q,
    double tol_x_rad,
    double tol_y_rad,
    double tol_z_rad,
    double weight = 1.0)
{
  moveit_msgs::msg::Constraints c;
  moveit_msgs::msg::OrientationConstraint oc;

  oc.link_name = link_name;
  oc.header.frame_id = frame_id;
  oc.orientation = q;

  oc.absolute_x_axis_tolerance = std::fabs(tol_x_rad);
  oc.absolute_y_axis_tolerance = std::fabs(tol_y_rad);
  oc.absolute_z_axis_tolerance = std::fabs(tol_z_rad);
  oc.weight = weight;

  c.orientation_constraints.push_back(oc);
  return c;
}

static geometry_msgs::msg::Vector3 v3(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

static geometry_msgs::msg::Vector3 v3_add(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& b)
{
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static geometry_msgs::msg::Vector3 v3_sub(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& b)
{
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static geometry_msgs::msg::Vector3 v3_scale(const geometry_msgs::msg::Vector3& a, double s)
{
  return v3(a.x * s, a.y * s, a.z * s);
}

static double v3_norm(const geometry_msgs::msg::Vector3& a)
{
  return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

static geometry_msgs::msg::Vector3 v3_unit(const geometry_msgs::msg::Vector3& a)
{
  const double n = v3_norm(a);
  if (n < 1e-12) return v3(0,0,0);
  return v3_scale(a, 1.0/n);
}

static geometry_msgs::msg::Vector3 v3_cross(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& b)
{
  return v3(a.y*b.z - a.z*b.y,
            a.z*b.x - a.x*b.z,
            a.x*b.y - a.y*b.x);
}

// ============================================================
// Logging helpers (verbose motion debug)
// ============================================================
struct MotionCtx
{
  std::string phase;   // "COARSE" / "FINE" / "VIEW" / "CART"
  int attempt{-1};     // coarse attempt, or -1
  int segment{-1};     // segmented iter, or -1
  int replan{-1};      // runtime replan count, or -1
  std::string tag;     // free tag like "pre", "shot", "turn"
};

static const char* tf_bool(bool v) { return v ? "true" : "false"; }

static void log_pose_xyz_q(const rclcpp::Logger& logger,
                           const std::string& name,
                           const geometry_msgs::msg::PoseStamped& ps)
{
  const auto& p = ps.pose.position;
  const auto& q = ps.pose.orientation;
  RCLCPP_INFO(logger,
    "%s: frame=%s pos=(%.3f,%.3f,%.3f) q=(%.4f,%.4f,%.4f,%.4f)",
    name.c_str(), ps.header.frame_id.c_str(),
    p.x,p.y,p.z, q.x,q.y,q.z,q.w);
}

static void log_tf_xyz_q(const rclcpp::Logger& logger,
                         const std::string& name,
                         const geometry_msgs::msg::TransformStamped& T)
{
  const auto& t = T.transform.translation;
  const auto& q = T.transform.rotation;
  RCLCPP_INFO(logger,
    "%s: parent=%s child=%s t=(%.3f,%.3f,%.3f) q=(%.4f,%.4f,%.4f,%.4f)",
    name.c_str(),
    T.header.frame_id.c_str(), T.child_frame_id.c_str(),
    t.x,t.y,t.z, q.x,q.y,q.z,q.w);
}

static void log_ctx_prefix(const rclcpp::Logger& logger, const MotionCtx& ctx, const char* msg)
{
  RCLCPP_INFO(logger, "[%s][a=%d][seg=%d][replan=%d][%s] %s",
              ctx.phase.c_str(), ctx.attempt, ctx.segment, ctx.replan,
              ctx.tag.c_str(), msg);
}

// Best-effort: read joint angle from current state
static bool get_joint_angle_best_effort(moveit::planning_interface::MoveGroupInterface& mg,
                                        const std::string& joint_name,
                                        double timeout_sec,
                                        double& out_rad)
{
  try {
    moveit::core::RobotStatePtr st = mg.getCurrentState(timeout_sec);
    if (!st) return false;

    out_rad = st->getVariablePosition(joint_name);
    return std::isfinite(out_rad);
  } catch (...) {
    return false;
  }
}

// ============================================================
// TF: lookupTransform(target, source) returns target_T_source
// NOTE: app_node executor thread is spinning, so we do NOT spin here.
// ============================================================
static bool lookup_tf_blocking(tf2_ros::Buffer& tf_buffer,
                               const rclcpp::Node::SharedPtr& node,
                               const std::string& target,
                               const std::string& source,
                               geometry_msgs::msg::TransformStamped& out,
                               double timeout_sec)
{
  const auto logger = node->get_logger();
  const auto clock  = node->get_clock();
  const auto start  = clock->now();
  rclcpp::Rate rate(50);

  while (rclcpp::ok()) {
    try {
      out = tf_buffer.lookupTransform(target, source, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
                           "Waiting TF %s -> %s: %s",
                           target.c_str(), source.c_str(), ex.what());
    }

    if ((clock->now() - start).seconds() > timeout_sec) {
      return false;
    }
    rate.sleep();
  }
  return false;
}

static geometry_msgs::msg::Vector3 pos_from_tf(const geometry_msgs::msg::TransformStamped &T)
{
  return v3(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
}

// ============================================================
// Best-effort RTAB-Map parameter switching.
// ============================================================
static bool set_rtabmap_params_best_effort(
    const rclcpp::Logger &logger,
    const std::shared_ptr<rclcpp::SyncParametersClient> &client,
    const std::vector<rclcpp::Parameter> &params,
    double wait_service_sec)
{
  if (!client) {
    RCLCPP_WARN(logger, "RTAB-Map params client is null.");
    return false;
  }

  if (!client->wait_for_service(sec_to_ns(wait_service_sec))) {
    RCLCPP_WARN(logger, "RTAB-Map set_parameters service not available (waited %.2fs).", wait_service_sec);
    return false;
  }

  const auto results = client->set_parameters(params);

  bool all_ok = true;
  for (size_t i = 0; i < results.size() && i < params.size(); ++i) {
    const auto &r = results[i];
    const auto &p = params[i];
    if (!r.successful) {
      all_ok = false;
      RCLCPP_WARN(logger, "RTAB-Map param set FAILED: %s (%s)",
                  p.get_name().c_str(), r.reason.c_str());
    } else {
      RCLCPP_INFO(logger, "RTAB-Map param set OK: %s", p.get_name().c_str());
    }
  }
  return all_ok;
}

// ============================================================
// PointCloud2 monitor
// - Detect close points in “camera optical frame”, +Z forward.
// ============================================================
class ForwardObstacleMonitor
{
public:
  ForwardObstacleMonitor(const rclcpp::Node::SharedPtr& node,
                         const std::string& pc_topic,
                         double roi_x_m, double roi_y_m,
                         double max_z_m)
  : node_(node),
    roi_x_m_(roi_x_m), roi_y_m_(roi_y_m), max_z_m_(max_z_m)
  {
    auto qos = rclcpp::SensorDataQoS();

    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        this->on_cloud(msg);
      });

    last_min_z_.store(std::numeric_limits<double>::infinity());
    last_stamp_sec_.store(0.0);
  }

  double last_min_forward_m() const { return last_min_z_.load(); }
  double last_stamp_sec() const { return last_stamp_sec_.load(); }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
  {
    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto& f : msg->fields) {
      if (f.name == "x") off_x = static_cast<int>(f.offset);
      else if (f.name == "y") off_y = static_cast<int>(f.offset);
      else if (f.name == "z") off_z = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) return;

    double min_z = std::numeric_limits<double>::infinity();

    const uint8_t* data = msg->data.data();
    const size_t   step = msg->point_step;
    const size_t   npts = (step > 0) ? (msg->data.size() / step) : 0;

    for (size_t i = 0; i < npts; ++i) {
      const uint8_t* p = data + i * step;

      float x = *reinterpret_cast<const float*>(p + off_x);
      float y = *reinterpret_cast<const float*>(p + off_y);
      float z = *reinterpret_cast<const float*>(p + off_z);

      if (!is_finite3(x, y, z)) continue;
      if (z <= 0.0f) continue;
      if (z > static_cast<float>(max_z_m_)) continue;

      if (std::fabs(x) > static_cast<float>(roi_x_m_)) continue;
      if (std::fabs(y) > static_cast<float>(roi_y_m_)) continue;

      if (z < min_z) min_z = z;
    }

    last_min_z_.store(min_z);

    double t = static_cast<double>(msg->header.stamp.sec) +
               1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    last_stamp_sec_.store(t);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  double roi_x_m_{0.15};
  double roi_y_m_{0.15};
  double max_z_m_{1.0};

  std::atomic<double> last_min_z_;
  std::atomic<double> last_stamp_sec_;
};

// ============================================================
// Speed scaling helper (distance-based)
// ============================================================
static void apply_distance_based_scaling(
    const rclcpp::Logger &logger,
    moveit::planning_interface::MoveGroupInterface &move_group,
    double dist_m,
    double th_slow_m,
    double th_very_slow_m,
    double v_nominal, double a_nominal,
    double v_slow,    double a_slow,
    double v_vslow,   double a_vslow)
{
  double v = v_nominal;
  double a = a_nominal;

  if (dist_m < th_very_slow_m) {
    v = v_vslow; a = a_vslow;
  } else if (dist_m < th_slow_m) {
    v = v_slow;  a = a_slow;
  }

  v = clamp01(v);
  a = clamp01(a);

  move_group.setMaxVelocityScalingFactor(v);
  move_group.setMaxAccelerationScalingFactor(a);

  RCLCPP_INFO(logger,
              "Speed scaling by distance: dist=%.3f[m], vel=%.3f, acc=%.3f",
              dist_m, v, a);
}

// ============================================================
// Look-at pose builder
// - Camera optical +Z points to target (leaf center).
// - “Up” is based on world/base_link +Z.
// ============================================================
static geometry_msgs::msg::Quaternion quat_from_R(const geometry_msgs::msg::Vector3& x_axis,
                                                  const geometry_msgs::msg::Vector3& y_axis,
                                                  const geometry_msgs::msg::Vector3& z_axis)
{
  const double r00 = x_axis.x, r01 = y_axis.x, r02 = z_axis.x;
  const double r10 = x_axis.y, r11 = y_axis.y, r12 = z_axis.y;
  const double r20 = x_axis.z, r21 = y_axis.z, r22 = z_axis.z;

  geometry_msgs::msg::Quaternion q;
  const double tr = r00 + r11 + r22;

  if (tr > 0.0) {
    const double S = std::sqrt(tr + 1.0) * 2.0;
    q.w = 0.25 * S;
    q.x = (r21 - r12) / S;
    q.y = (r02 - r20) / S;
    q.z = (r10 - r01) / S;
  } else if ((r00 > r11) && (r00 > r22)) {
    const double S = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    q.w = (r21 - r12) / S;
    q.x = 0.25 * S;
    q.y = (r01 + r10) / S;
    q.z = (r02 + r20) / S;
  } else if (r11 > r22) {
    const double S = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    q.w = (r02 - r20) / S;
    q.x = (r01 + r10) / S;
    q.y = 0.25 * S;
    q.z = (r12 + r21) / S;
  } else {
    const double S = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    q.w = (r10 - r01) / S;
    q.x = (r02 + r20) / S;
    q.y = (r12 + r21) / S;
    q.z = 0.25 * S;
  }
  return q;
}

static geometry_msgs::msg::PoseStamped make_lookat_pose(
    const std::string& frame_id,
    const geometry_msgs::msg::Vector3& target_pos_world,
    const geometry_msgs::msg::Vector3& world_up,
    const geometry_msgs::msg::Vector3& desired_pos_world)
{
  geometry_msgs::msg::Vector3 z = v3_sub(target_pos_world, desired_pos_world);
  z = v3_unit(z);
  if (v3_norm(z) < 1e-9) z = v3(0, 0, 1);

  geometry_msgs::msg::Vector3 x = v3_cross(world_up, z);
  if (v3_norm(x) < 1e-9) x = v3_cross(v3(0,1,0), z);
  x = v3_unit(x);

  geometry_msgs::msg::Vector3 y = v3_cross(z, x);
  y = v3_unit(y);

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame_id;
  p.pose.position.x = desired_pos_world.x;
  p.pose.position.y = desired_pos_world.y;
  p.pose.position.z = desired_pos_world.z;
  p.pose.orientation = quat_from_R(x, y, z);
  return p;
}

// ============================================================
// Helper: choose stop distance + leaf-self ignore based on dist_to_leaf
// ============================================================
static void compute_stop_policy(
    const rclcpp::Logger& logger,
    const rclcpp::Clock::SharedPtr& clock,
    double min_forward,
    double dist_to_leaf,
    double stop_dist_default,
    bool enable_goal_proximity_gating,
    double dist_mid_m,
    double dist_near_m,
    double stop_dist_mid_m,
    double stop_dist_near_m,
    bool enable_leaf_self_ignore,
    double leaf_ignore_within_m,
    double leaf_ignore_band_m,
    double& stop_dist_eff_out,
    bool& ignore_as_leaf_out)
{
  double stop_dist_eff = stop_dist_default;
  if (enable_goal_proximity_gating && std::isfinite(dist_to_leaf)) {
    if (dist_to_leaf < dist_near_m) stop_dist_eff = stop_dist_near_m;
    else if (dist_to_leaf < dist_mid_m) stop_dist_eff = stop_dist_mid_m;
  }

  bool ignore_as_leaf = false;
  if (enable_leaf_self_ignore &&
      std::isfinite(dist_to_leaf) && dist_to_leaf < leaf_ignore_within_m &&
      std::isfinite(min_forward)) {
    if (std::fabs(min_forward - dist_to_leaf) < leaf_ignore_band_m) {
      ignore_as_leaf = true;
      RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
        "Monitor: ignore leaf-as-obstacle (min_z=%.3f, dist_to_leaf=%.3f, band=%.3f).",
        min_forward, dist_to_leaf, leaf_ignore_band_m);
    }
  }

  stop_dist_eff_out = stop_dist_eff;
  ignore_as_leaf_out = ignore_as_leaf;
}

// ============================================================
// Plan + Execute with monitoring (verbose logging + runtime replan)
// NOTE: app_node executor thread is spinning, so we do NOT spin here.
// ============================================================
static bool plan_execute_with_runtime_replan(
    const rclcpp::Node::SharedPtr& app_node,
    const rclcpp::Logger& logger,
    tf2_ros::Buffer& tf_buffer,
    const std::string& world_frame,
    const std::string& leaf_center_frame,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::string& ee_link_for_target,
    ForwardObstacleMonitor* obstacle_monitor,
    bool enable_runtime_monitor,
    double monitor_rate_hz,
    double stop_dist_m,
    double pc_stale_sec,
    int max_runtime_replans,
    double replan_backoff_sec,
    bool enable_goal_proximity_gating,
    double dist_mid_m,
    double dist_near_m,
    double stop_dist_mid_m,
    double stop_dist_near_m,
    bool enable_leaf_self_ignore,
    double leaf_ignore_within_m,
    double leaf_ignore_band_m,
    const MotionCtx& ctx,
    bool verbose_motion_log,
    int monitor_log_every_n,
    bool log_on_stop_event,
    bool log_wrist1_angle,
    const std::string& wrist1_joint_name)
{
  int replans = 0;

  while (rclcpp::ok()) {
    if (verbose_motion_log) {
      MotionCtx c = ctx;
      c.replan = replans;

      log_ctx_prefix(logger, c, "PLAN begin (snapshot)");

      const auto cur_ps = move_group.getCurrentPose(ee_link_for_target);
      geometry_msgs::msg::PoseStamped cur = cur_ps;
      cur.header.frame_id = world_frame; // display only
      log_pose_xyz_q(logger, "  cur_pose", cur);
      log_pose_xyz_q(logger, "  goal_pose", goal);

      geometry_msgs::msg::TransformStamped Tw_leaf, Tw_cam;
      bool have_leaf=false, have_cam=false;
      try { Tw_leaf = tf_buffer.lookupTransform(world_frame, leaf_center_frame, tf2::TimePointZero); have_leaf=true; } catch(...) {}
      try { Tw_cam  = tf_buffer.lookupTransform(world_frame, ee_link_for_target, tf2::TimePointZero); have_cam=true; } catch(...) {}

      if (have_leaf) log_tf_xyz_q(logger, "  Tw_leaf", Tw_leaf);
      if (have_cam)  log_tf_xyz_q(logger, "  Tw_cam",  Tw_cam);

      if (have_leaf && have_cam) {
        const double d = dist_xyz(Tw_cam.transform.translation, Tw_leaf.transform.translation);
        RCLCPP_INFO(logger, "  dist(cam,leaf)=%.3f", d);
      }

      if (enable_runtime_monitor && obstacle_monitor) {
        const double now = app_node->get_clock()->now().seconds();
        const double pc_stamp = obstacle_monitor->last_stamp_sec();
        const double min_forward = obstacle_monitor->last_min_forward_m();
        const double age = (pc_stamp > 0.0) ? (now - pc_stamp) : std::numeric_limits<double>::infinity();
        const bool stale = (pc_stamp <= 0.0) || (age > pc_stale_sec);
        RCLCPP_INFO(logger, "  monitor: min_z=%.3f pc_age=%.3f stale=%s",
                    min_forward, age, tf_bool(stale));
      }

      if (log_wrist1_angle) {
        double w1 = 0.0;
        if (get_joint_angle_best_effort(move_group, wrist1_joint_name, 0.1, w1)) {
          RCLCPP_INFO(logger, "  joint[%s]=%.3frad (%.1fdeg)",
                      wrist1_joint_name.c_str(), w1, w1*180.0/M_PI);
        } else {
          RCLCPP_INFO(logger, "  joint[%s]=<unavailable>", wrist1_joint_name.c_str());
        }
      }
    }

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal, ee_link_for_target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto t_plan0 = std::chrono::steady_clock::now();
    const bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    auto t_plan1 = std::chrono::steady_clock::now();
    const double plan_ms = 1e3 * std::chrono::duration<double>(t_plan1 - t_plan0).count();

    {
      MotionCtx c = ctx;
      c.replan = replans;
      if (!planned) {
        RCLCPP_ERROR(logger,
          "[%s][a=%d][seg=%d][replan=%d][%s] PLAN failed (%.1fms) EE=%s goal_frame=%s",
          c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(),
          plan_ms, ee_link_for_target.c_str(), goal.header.frame_id.c_str());
        return false;
      } else if (verbose_motion_log) {
        RCLCPP_INFO(logger,
          "[%s][a=%d][seg=%d][replan=%d][%s] PLAN ok (%.1fms)",
          c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(), plan_ms);
      }
    }

    std::atomic<bool> exec_done{false};
    std::atomic<int>  exec_code_val{0};
    std::atomic<bool> stop_requested{false};

    auto t_exec0 = std::chrono::steady_clock::now();

    std::thread exec_thread([&](){
      MotionCtx c = ctx; c.replan = replans;
      RCLCPP_INFO(logger, "[%s][a=%d][seg=%d][replan=%d][%s] Executing plan...",
                  c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str());
      auto ret = move_group.execute(plan);
      exec_code_val.store(ret.val);
      exec_done.store(true);
    });

    rclcpp::Rate rate(std::max(1.0, monitor_rate_hz));
    const auto clock = app_node->get_clock();
    int mon_iter = 0;

    while (rclcpp::ok() && !exec_done.load()) {
      mon_iter++;

      if (enable_runtime_monitor && obstacle_monitor) {
        const double now = clock->now().seconds();
        const double pc_stamp = obstacle_monitor->last_stamp_sec();
        const double min_forward = obstacle_monitor->last_min_forward_m();
        const double age = (pc_stamp > 0.0) ? (now - pc_stamp) : std::numeric_limits<double>::infinity();
        const bool stale = (pc_stamp <= 0.0) || (age > pc_stale_sec);

        double dist_to_leaf = std::numeric_limits<double>::infinity();
        bool have_leaf = false, have_cam = false;
        geometry_msgs::msg::TransformStamped Tw_leaf, Tw_cam;

        try { Tw_leaf = tf_buffer.lookupTransform(world_frame, leaf_center_frame, tf2::TimePointZero); have_leaf = true; } catch (...) {}
        try { Tw_cam  = tf_buffer.lookupTransform(world_frame, ee_link_for_target, tf2::TimePointZero); have_cam  = true; } catch (...) {}

        if (have_leaf && have_cam) {
          dist_to_leaf = dist_xyz(Tw_cam.transform.translation, Tw_leaf.transform.translation);
        }

        double stop_dist_eff = stop_dist_m;
        bool ignore_as_leaf = false;

        if (!stale && std::isfinite(min_forward)) {
          compute_stop_policy(
            logger, clock,
            min_forward, dist_to_leaf,
            stop_dist_m,
            enable_goal_proximity_gating,
            dist_mid_m, dist_near_m,
            stop_dist_mid_m, stop_dist_near_m,
            enable_leaf_self_ignore,
            leaf_ignore_within_m, leaf_ignore_band_m,
            stop_dist_eff, ignore_as_leaf);
        }

        if (verbose_motion_log && monitor_log_every_n > 0 && (mon_iter % monitor_log_every_n == 0)) {
          MotionCtx c = ctx; c.replan = replans;
          RCLCPP_INFO(logger,
            "[%s][a=%d][seg=%d][replan=%d][%s] MON: min_z=%.3f age=%.3f stale=%s dist_to_leaf=%.3f stop_eff=%.3f ignore_leaf=%s",
            c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(),
            min_forward, age, tf_bool(stale), dist_to_leaf, stop_dist_eff, tf_bool(ignore_as_leaf));
        }

        if (!stale && std::isfinite(min_forward) && !ignore_as_leaf && (min_forward < stop_dist_eff)) {
          MotionCtx c = ctx; c.replan = replans;
          if (log_on_stop_event) {
            RCLCPP_WARN(logger,
              "[%s][a=%d][seg=%d][replan=%d][%s] STOP: min_z=%.3f < stop_eff=%.3f dist_to_leaf=%.3f age=%.3f",
              c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(),
              min_forward, stop_dist_eff, dist_to_leaf, age);
          }
          stop_requested.store(true);
          move_group.stop();
          break;
        }
      }

      rate.sleep();
    }

    if (exec_thread.joinable()) exec_thread.join();

    auto t_exec1 = std::chrono::steady_clock::now();
    const double exec_ms = 1e3 * std::chrono::duration<double>(t_exec1 - t_exec0).count();

    const int code = exec_code_val.load();

    if (stop_requested.load()) {
      MotionCtx c = ctx; c.replan = replans;
      RCLCPP_WARN(logger,
        "[%s][a=%d][seg=%d][replan=%d][%s] EXEC interrupted by STOP (%.1fms). replan %d/%d backoff=%.2fs",
        c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(),
        exec_ms, (replans + 1), max_runtime_replans, replan_backoff_sec);

      replans++;
      if (replans > max_runtime_replans) {
        RCLCPP_ERROR(logger,
          "[%s][a=%d][seg=%d][replan=%d][%s] Exceeded max runtime replans (%d). Give up this goal.",
          c.phase.c_str(), c.attempt, c.segment, replans, c.tag.c_str(), max_runtime_replans);
        return false;
      }
      rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
      continue;
    }

    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      MotionCtx c = ctx; c.replan = replans;
      RCLCPP_ERROR(logger,
        "[%s][a=%d][seg=%d][replan=%d][%s] EXEC failed code=%d (%.1fms). retry %d/%d backoff=%.2fs",
        c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(),
        code, exec_ms, (replans + 1), max_runtime_replans, replan_backoff_sec);

      replans++;
      if (replans > max_runtime_replans) {
        RCLCPP_ERROR(logger,
          "[%s][a=%d][seg=%d][replan=%d][%s] Exceeded max retries after execution failure (%d).",
          c.phase.c_str(), c.attempt, c.segment, replans, c.tag.c_str(), max_runtime_replans);
        return false;
      }

      rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
      continue;
    }

    {
      MotionCtx c = ctx; c.replan = replans;
      RCLCPP_INFO(logger,
        "[%s][a=%d][seg=%d][replan=%d][%s] EXEC success (%.1fms)",
        c.phase.c_str(), c.attempt, c.segment, c.replan, c.tag.c_str(), exec_ms);
    }
    return true;
  }

  return false;
}

// ============================================================
// Execute RobotTrajectory with monitoring (Cartesian path)
// ============================================================
static bool execute_traj_with_monitor_gated(
    const rclcpp::Node::SharedPtr& app_node,
    const rclcpp::Logger& logger,
    tf2_ros::Buffer& tf_buffer,
    const std::string& world_frame,
    const std::string& leaf_center_frame,
    const std::string& ee_link_for_target,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::RobotTrajectory& traj,
    ForwardObstacleMonitor* obstacle_monitor,
    bool enable_runtime_monitor,
    double monitor_rate_hz,
    double stop_dist_m,
    double pc_stale_sec,
    double replan_backoff_sec,
    bool enable_goal_proximity_gating,
    double dist_mid_m,
    double dist_near_m,
    double stop_dist_mid_m,
    double stop_dist_near_m,
    bool enable_leaf_self_ignore,
    double leaf_ignore_within_m,
    double leaf_ignore_band_m)
{
  std::atomic<bool> exec_done{false};
  std::atomic<int>  exec_code_val{0};
  std::atomic<bool> stop_requested{false};

  std::thread exec_thread([&](){
    RCLCPP_INFO(logger, "Executing Cartesian trajectory...");
    auto ret = move_group.execute(traj);
    exec_code_val.store(ret.val);
    exec_done.store(true);
  });

  rclcpp::Rate rate(std::max(1.0, monitor_rate_hz));
  const auto clock = app_node->get_clock();

  while (rclcpp::ok() && !exec_done.load()) {
    if (enable_runtime_monitor && obstacle_monitor) {
      const double now = clock->now().seconds();
      const double pc_stamp = obstacle_monitor->last_stamp_sec();
      const double min_forward = obstacle_monitor->last_min_forward_m();
      const bool stale = (pc_stamp <= 0.0) || ((now - pc_stamp) > pc_stale_sec);

      double dist_to_leaf = std::numeric_limits<double>::infinity();
      bool have_leaf = false, have_cam = false;
      geometry_msgs::msg::TransformStamped Tw_leaf, Tw_cam;

      try { Tw_leaf = tf_buffer.lookupTransform(world_frame, leaf_center_frame, tf2::TimePointZero); have_leaf = true; } catch (...) {}
      try { Tw_cam  = tf_buffer.lookupTransform(world_frame, ee_link_for_target, tf2::TimePointZero); have_cam  = true; } catch (...) {}

      if (have_leaf && have_cam) {
        dist_to_leaf = dist_xyz(Tw_cam.transform.translation, Tw_leaf.transform.translation);
      }

      if (!stale && std::isfinite(min_forward)) {
        double stop_dist_eff = stop_dist_m;
        bool ignore_as_leaf = false;

        compute_stop_policy(
          logger, clock,
          min_forward, dist_to_leaf,
          stop_dist_m,
          enable_goal_proximity_gating,
          dist_mid_m, dist_near_m,
          stop_dist_mid_m, stop_dist_near_m,
          enable_leaf_self_ignore,
          leaf_ignore_within_m, leaf_ignore_band_m,
          stop_dist_eff, ignore_as_leaf);

        if (!ignore_as_leaf && (min_forward < stop_dist_eff)) {
          RCLCPP_WARN(logger,
            "Obstacle too close during Cartesian: min_z=%.3f < stop_eff=%.3f (dist_to_leaf=%.3f). STOP.",
            min_forward, stop_dist_eff, dist_to_leaf);
          stop_requested.store(true);
          move_group.stop();
          break;
        }
      }
    }

    rate.sleep();
  }

  if (exec_thread.joinable()) exec_thread.join();

  if (stop_requested.load()) {
    rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
    return false;
  }

  const int code = exec_code_val.load();
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Cartesian execution failed (MoveItErrorCode=%d).", code);
    rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
    return false;
  }

  RCLCPP_INFO(logger, "Cartesian execution SUCCESS.");
  return true;
}

// ============================================================
// Segmented motion helper (ctx-aware logging)
// - Late EE orientation path-constraint (optional)
// ============================================================
static bool move_segmented_lookat(
    const rclcpp::Node::SharedPtr& app_node,
    const rclcpp::Logger& logger,
    tf2_ros::Buffer& tf_buffer,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& world_frame,
    const std::string& ee_link,
    const std::string& leaf_center_frame,
    const geometry_msgs::msg::Vector3& final_pos_world,
    double segment_max_m,
    double arrive_pos_tol_m,
    ForwardObstacleMonitor* obstacle_monitor,
    bool enable_runtime_monitor,
    double monitor_rate_hz,
    double stop_dist_m,
    double pc_stale_sec,
    int max_runtime_replans_per_segment,
    double replan_backoff_sec,
    double tf_timeout_sec,
    bool enable_goal_proximity_gating,
    double dist_mid_m,
    double dist_near_m,
    double stop_dist_mid_m,
    double stop_dist_near_m,
    bool enable_leaf_self_ignore,
    double leaf_ignore_within_m,
    double leaf_ignore_band_m,
    const std::string& phase,
    int attempt,
    const std::string& tag,
    bool verbose_motion_log,
    int monitor_log_every_n,
    bool log_on_stop_event,
    bool log_wrist1_angle,
    const std::string& wrist1_joint_name,
    bool enable_late_ee_orientation_constraint,
    int coarse_orient_from_seg,
    int fine_orient_from_seg,
    double orient_enable_rem_m,
    double ee_orient_tol_x_rad,
    double ee_orient_tol_y_rad,
    double ee_orient_tol_z_rad,
    double ee_orient_weight)
{
  const geometry_msgs::msg::Vector3 world_up = v3(0,0,1);

  geometry_msgs::msg::TransformStamped Tw_leaf;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Tw_leaf, tf_timeout_sec)) {
    RCLCPP_ERROR(logger, "Cannot get TF to leaf center: %s", leaf_center_frame.c_str());
    return false;
  }
  geometry_msgs::msg::Vector3 leaf_pos_world = pos_from_tf(Tw_leaf);

  RCLCPP_INFO(logger,
    "[SEG] start: final_pos=(%.3f,%.3f,%.3f) leaf=(%.3f,%.3f,%.3f) seg_max=%.3f tol=%.3f",
    final_pos_world.x, final_pos_world.y, final_pos_world.z,
    leaf_pos_world.x,  leaf_pos_world.y,  leaf_pos_world.z,
    segment_max_m, arrive_pos_tol_m);

  bool ever_set_orient_constraint = false;

  for (int iter = 0; rclcpp::ok(); ++iter) {
    const auto cur_ps = move_group.getCurrentPose(ee_link);
    geometry_msgs::msg::Vector3 cur_pos = v3(cur_ps.pose.position.x, cur_ps.pose.position.y, cur_ps.pose.position.z);

    const geometry_msgs::msg::Vector3 diff = v3_sub(final_pos_world, cur_pos);
    const double rem = v3_norm(diff);

    RCLCPP_INFO(logger,
      "[SEG %d] cur=(%.3f,%.3f,%.3f) leaf=(%.3f,%.3f,%.3f) final=(%.3f,%.3f,%.3f) rem=%.3f",
      iter,
      cur_pos.x, cur_pos.y, cur_pos.z,
      leaf_pos_world.x, leaf_pos_world.y, leaf_pos_world.z,
      final_pos_world.x, final_pos_world.y, final_pos_world.z,
      rem);

    if (rem <= arrive_pos_tol_m) {
      if (ever_set_orient_constraint) {
        move_group.clearPathConstraints();
      }
      RCLCPP_INFO(logger, "Segmented move reached tolerance: rem=%.4f <= tol=%.4f", rem, arrive_pos_tol_m);
      return true;
    }

    geometry_msgs::msg::Vector3 seg_pos = final_pos_world;
    if (rem > segment_max_m) {
      seg_pos = v3_add(cur_pos, v3_scale(v3_unit(diff), segment_max_m));
    }

    geometry_msgs::msg::PoseStamped seg_goal =
      make_lookat_pose(world_frame, leaf_pos_world, world_up, seg_pos);

    MotionCtx ctx;
    ctx.phase   = phase;
    ctx.attempt = attempt;
    ctx.segment = iter;
    ctx.replan  = 0;
    ctx.tag     = tag;

    bool apply_orient_constraint = false;
    if (enable_late_ee_orientation_constraint) {
      const bool phase_is_coarse = (phase == "COARSE");
      const bool phase_is_fine   = (phase == "FINE");

      const int from_seg = phase_is_coarse ? coarse_orient_from_seg
                                           : (phase_is_fine ? fine_orient_from_seg : 999999);

      if ((iter >= from_seg) || (rem <= orient_enable_rem_m)) {
        apply_orient_constraint = true;
      }
    }

    if (apply_orient_constraint) {
      const auto c = make_ee_orientation_constraint(
        ee_link, world_frame, seg_goal.pose.orientation,
        ee_orient_tol_x_rad, ee_orient_tol_y_rad, ee_orient_tol_z_rad,
        ee_orient_weight);
      move_group.setPathConstraints(c);
      ever_set_orient_constraint = true;

      if (verbose_motion_log) {
        RCLCPP_INFO(logger,
          "[%s][a=%d][seg=%d][%s] EE Orientation PathConstraint ENABLED (tol_deg=%.1f/%.1f/%.1f, rem=%.3f)",
          phase.c_str(), attempt, iter, tag.c_str(),
          ee_orient_tol_x_rad*180.0/M_PI, ee_orient_tol_y_rad*180.0/M_PI, ee_orient_tol_z_rad*180.0/M_PI,
          rem);
      }
    } else {
      if (enable_late_ee_orientation_constraint) {
        move_group.clearPathConstraints();
      }
    }

    const bool ok = plan_execute_with_runtime_replan(
      app_node, logger,
      tf_buffer, world_frame, leaf_center_frame,
      move_group,
      seg_goal, ee_link,
      obstacle_monitor,
      enable_runtime_monitor,
      monitor_rate_hz,
      stop_dist_m,
      pc_stale_sec,
      max_runtime_replans_per_segment,
      replan_backoff_sec,
      enable_goal_proximity_gating,
      dist_mid_m, dist_near_m,
      stop_dist_mid_m, stop_dist_near_m,
      enable_leaf_self_ignore,
      leaf_ignore_within_m,
      leaf_ignore_band_m,
      ctx,
      verbose_motion_log,
      monitor_log_every_n,
      log_on_stop_event,
      log_wrist1_angle,
      wrist1_joint_name);

    if (!ok) {
      if (ever_set_orient_constraint) {
        move_group.clearPathConstraints();
      }
      RCLCPP_WARN(logger, "Segmented execution failed for a segment.");
      return false;
    }

    if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Tw_leaf, tf_timeout_sec)) {
      RCLCPP_WARN(logger, "Leaf TF refresh failed; keeping previous leaf center.");
    } else {
      leaf_pos_world = pos_from_tf(Tw_leaf);
    }
  }

  if (ever_set_orient_constraint) {
    move_group.clearPathConstraints();
  }
  return false;
}

// ============================================================
// Compute COARSE pre-position: “down + back”
// ============================================================
static geometry_msgs::msg::Vector3 compute_leaf_pre_position(
    const geometry_msgs::msg::Vector3& cam_pos_world,
    const geometry_msgs::msg::Vector3& leaf_pos_world,
    double pre_back_m,
    double pre_down_m,
    double min_xy_norm_eps = 1e-6)
{
  geometry_msgs::msg::Vector3 v = v3_sub(leaf_pos_world, cam_pos_world);

  geometry_msgs::msg::Vector3 vxy = v3(v.x, v.y, 0.0);
  const double nxy = v3_norm(vxy);

  geometry_msgs::msg::Vector3 dir_xy = v3(1, 0, 0);
  if (nxy > min_xy_norm_eps) dir_xy = v3_scale(vxy, 1.0 / nxy);

  geometry_msgs::msg::Vector3 pre = leaf_pos_world;
  pre = v3_sub(pre, v3_scale(dir_xy, pre_back_m));
  pre = v3_sub(pre, v3(0,0,pre_down_m));
  return pre;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ============================================================
  // Node split
  // ============================================================
  // app_node keeps the original name so your CLI parameters keep working.
  auto app_node = rclcpp::Node::make_shared("coarse_to_fine_go_to_leaf3");
  const auto logger = app_node->get_logger();
  const auto clock  = app_node->get_clock();

  // moveit_node is a separate node for MoveGroupInterface.
  // DO NOT add it to our executor. (MoveGroupInterface will manage its own internal executor/spin.)
  auto moveit_node = rclcpp::Node::make_shared("coarse_to_fine_go_to_leaf3_moveit");

  // ------------------------------------------------------------
  // Keep spinning app_node in a dedicated thread
  // ------------------------------------------------------------
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(app_node);

  std::thread spin_thread([&](){
    exec.spin();
  });

  auto cleanup_and_exit = [&](int code)->int {
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return code;
  };

  // use_sim_time: declare on app_node (CLI override is applied here)
  if (!app_node->has_parameter("use_sim_time")) {
    app_node->declare_parameter<bool>("use_sim_time", true);
  }
  bool use_sim_time = true;
  app_node->get_parameter("use_sim_time", use_sim_time);
  RCLCPP_INFO(logger, "use_sim_time=%s", use_sim_time ? "true" : "false");

  // Propagate to moveit_node (important in Gazebo / /clock environments)
  if (!moveit_node->has_parameter("use_sim_time")) {
    moveit_node->declare_parameter<bool>("use_sim_time", use_sim_time);
  }
  moveit_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

  // ============================================================
  // Parameters (on app_node)
  // ============================================================
  const std::string world_frame  = app_node->declare_parameter<std::string>("world_frame", "base_link");
  const std::string camera_frame = app_node->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  const std::string plant_frame  = app_node->declare_parameter<std::string>("plant_frame", "plant_base");

  const std::string leaf_center_frame = app_node->declare_parameter<std::string>("leaf_center_frame", "leaf_target");
  const std::string fine_goal_frame   = app_node->declare_parameter<std::string>("fine_goal_frame", "leaf_target_shot");
  const std::string view_goal_frame   = app_node->declare_parameter<std::string>("view_goal_frame", "leaf_target_view");

  const std::string ee_link = app_node->declare_parameter<std::string>("ee_link", "camera_color_optical_frame");

  const double switch_goal_m  = app_node->declare_parameter<double>("switch_goal_m", 0.25);
  const double switch_plant_m = app_node->declare_parameter<double>("switch_plant_m", 0.35);

  const int    max_coarse_attempts = app_node->declare_parameter<int>("max_coarse_attempts", 3);
  const double tf_timeout_sec      = app_node->declare_parameter<double>("tf_timeout_sec", 10.0);

  const double coarse_plan_time = app_node->declare_parameter<double>("coarse_planning_time", 2.0);
  const double fine_plan_time   = app_node->declare_parameter<double>("fine_planning_time",   5.0);
  const double view_plan_time   = app_node->declare_parameter<double>("view_planning_time",   3.0);

  const double coarse_vel = app_node->declare_parameter<double>("coarse_vel_scale", 0.02);
  const double coarse_acc = app_node->declare_parameter<double>("coarse_acc_scale", 0.02);
  const double fine_vel   = app_node->declare_parameter<double>("fine_vel_scale",   0.03);
  const double fine_acc   = app_node->declare_parameter<double>("fine_acc_scale",   0.03);
  const double view_vel   = app_node->declare_parameter<double>("view_vel_scale",   0.01);
  const double view_acc   = app_node->declare_parameter<double>("view_acc_scale",   0.01);

  const bool enable_distance_based_slowdown =
      app_node->declare_parameter<bool>("enable_distance_based_slowdown", true);

  const double th_slow_m       = app_node->declare_parameter<double>("slowdown_th_slow_m", 0.50);
  const double th_very_slow_m  = app_node->declare_parameter<double>("slowdown_th_very_slow_m", 0.30);

  const double coarse_vel_slow  = app_node->declare_parameter<double>("coarse_vel_scale_slow", 0.015);
  const double coarse_acc_slow  = app_node->declare_parameter<double>("coarse_acc_scale_slow", 0.015);
  const double coarse_vel_vslow = app_node->declare_parameter<double>("coarse_vel_scale_very_slow", 0.008);
  const double coarse_acc_vslow = app_node->declare_parameter<double>("coarse_acc_scale_very_slow", 0.008);

  const double coarse_retry_wait_sec =
      app_node->declare_parameter<double>("coarse_retry_wait_sec", 0.5);

  const double pre_back_m = app_node->declare_parameter<double>("pre_back_m", 0.25);
  const double pre_down_m = app_node->declare_parameter<double>("pre_down_m", 0.20);

  const bool   enable_segmented_motion = app_node->declare_parameter<bool>("enable_segmented_motion", true);
  const double segment_max_m           = app_node->declare_parameter<double>("segment_max_m", 0.15);
  const double arrive_pos_tol_m        = app_node->declare_parameter<double>("arrive_pos_tol_m", 0.02);

  const bool   enable_fine_cartesian   = app_node->declare_parameter<bool>("enable_fine_cartesian", true);
  const double cart_eef_step           = app_node->declare_parameter<double>("cart_eef_step", 0.01);
  const double cart_jump_thr           = app_node->declare_parameter<double>("cart_jump_threshold", 0.0);
  const double cart_min_fraction       = app_node->declare_parameter<double>("cart_min_fraction", 0.80);

  const bool enable_runtime_monitor =
      app_node->declare_parameter<bool>("enable_runtime_monitor", true);

  const std::string pc_topic =
      app_node->declare_parameter<std::string>("pointcloud_topic", "/camera/depth/points");

  const double monitor_rate_hz =
      app_node->declare_parameter<double>("monitor_rate_hz", 30.0);

  const double stop_dist_m =
      app_node->declare_parameter<double>("stop_dist_m", 0.35);

  const bool enable_goal_proximity_gating =
      app_node->declare_parameter<bool>("enable_goal_proximity_gating", true);

  const double dist_mid_m =
      app_node->declare_parameter<double>("dist_mid_m", 0.50);
  const double dist_near_m =
      app_node->declare_parameter<double>("dist_near_m", 0.25);

  const double stop_dist_mid_m =
      app_node->declare_parameter<double>("stop_dist_mid_m", 0.28);
  const double stop_dist_near_m =
      app_node->declare_parameter<double>("stop_dist_near_m", 0.18);

  const bool enable_leaf_self_ignore =
      app_node->declare_parameter<bool>("enable_leaf_self_ignore", true);

  const double leaf_ignore_within_m =
      app_node->declare_parameter<double>("leaf_ignore_within_m", 0.35);
  const double leaf_ignore_band_m =
      app_node->declare_parameter<double>("leaf_ignore_band_m", 0.06);

  const double roi_x_m =
      app_node->declare_parameter<double>("pc_roi_x_m", 0.18);
  const double roi_y_m =
      app_node->declare_parameter<double>("pc_roi_y_m", 0.18);

  const double pc_max_z_m =
      app_node->declare_parameter<double>("pc_max_z_m", 1.50);

  const double pc_stale_sec =
      app_node->declare_parameter<double>("pc_stale_sec", 0.20);

  const int max_runtime_replans =
      app_node->declare_parameter<int>("max_runtime_replans", 5);

  const double replan_backoff_sec =
      app_node->declare_parameter<double>("replan_backoff_sec", 0.30);

  // RTAB-Map param switching (optional)
  const bool   switch_rtabmap         = app_node->declare_parameter<bool>("switch_rtabmap", true);
  const std::string rtabmap_node_name = app_node->declare_parameter<std::string>("rtabmap_node_name", "/rtabmap");
  const double wait_param_service_sec = app_node->declare_parameter<double>("wait_param_service_sec", 5.0);
  const double fine_settle_wait_sec   = app_node->declare_parameter<double>("fine_settle_wait_sec", 2.0);

  const double coarse_detection_rate = app_node->declare_parameter<double>("coarse_detection_rate", 2.0);
  const double fine_detection_rate   = app_node->declare_parameter<double>("fine_detection_rate", 10.0);

  const bool set_grid_params      = app_node->declare_parameter<bool>("set_grid_params", false);
  const double coarse_grid_cell   = app_node->declare_parameter<double>("coarse_grid_cell", 0.05);
  const double fine_grid_cell     = app_node->declare_parameter<double>("fine_grid_cell",   0.02);
  const double coarse_range_max   = app_node->declare_parameter<double>("coarse_range_max", 3.0);
  const double fine_range_max     = app_node->declare_parameter<double>("fine_range_max",   1.5);

  const bool verbose_motion_log =
    app_node->declare_parameter<bool>("verbose_motion_log", true);

  const int monitor_log_every_n =
      app_node->declare_parameter<int>("monitor_log_every_n", 10);

  const bool log_on_stop_event =
      app_node->declare_parameter<bool>("log_on_stop_event", true);

  const bool log_wrist1_angle =
      app_node->declare_parameter<bool>("log_wrist1_angle", true);

  // Wrist_1 joint constraint (PathConstraints)
  const bool enable_wrist1_constraint =
      app_node->declare_parameter<bool>("enable_wrist1_constraint", false);

  const std::string wrist1_joint_name =
      app_node->declare_parameter<std::string>("wrist1_joint_name", "wrist_1_joint");

  const double wrist1_center_deg =
      app_node->declare_parameter<double>("wrist1_center_deg", 225.0);

  const double wrist1_tolerance_deg =
      app_node->declare_parameter<double>("wrist1_tolerance_deg", 45.0);

  // EE orientation path constraint (late segments)
  const bool enable_late_ee_orientation_constraint =
      app_node->declare_parameter<bool>("enable_late_ee_orientation_constraint", true);

  const int coarse_orient_from_seg =
      app_node->declare_parameter<int>("coarse_orient_from_seg", 4);
  const int fine_orient_from_seg =
      app_node->declare_parameter<int>("fine_orient_from_seg", 0);

  const double orient_enable_rem_m =
      app_node->declare_parameter<double>("orient_enable_rem_m", 0.20);

  const double ee_orient_tol_x_deg =
      app_node->declare_parameter<double>("ee_orient_tol_x_deg", 8.0);
  const double ee_orient_tol_y_deg =
      app_node->declare_parameter<double>("ee_orient_tol_y_deg", 8.0);
  const double ee_orient_tol_z_deg =
      app_node->declare_parameter<double>("ee_orient_tol_z_deg", 12.0);

  const double ee_orient_weight =
      app_node->declare_parameter<double>("ee_orient_weight", 1.0);

  const double ee_orient_tol_x_rad = deg2rad(ee_orient_tol_x_deg);
  const double ee_orient_tol_y_rad = deg2rad(ee_orient_tol_y_deg);
  const double ee_orient_tol_z_rad = deg2rad(ee_orient_tol_z_deg);

  // ============================================================
  // TF (app_node)
  // ============================================================
  tf2_ros::Buffer tf_buffer(clock);
  tf2_ros::TransformListener tf_listener(tf_buffer, app_node, false);

  // ============================================================
  // Point cloud obstacle monitor (app_node)
  // ============================================================
  ForwardObstacleMonitor obstacle_monitor(app_node, pc_topic, roi_x_m, roi_y_m, pc_max_z_m);

  RCLCPP_INFO(logger, "Runtime monitor: %s", enable_runtime_monitor ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(logger, "  pc_topic=%s stop_dist=%.3f roi_x=%.3f roi_y=%.3f max_z=%.3f stale=%.3f rate=%.1f",
              pc_topic.c_str(), stop_dist_m, roi_x_m, roi_y_m, pc_max_z_m, pc_stale_sec, monitor_rate_hz);

  // ============================================================
  // MoveIt (moveit_node)
  // ============================================================
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(moveit_node, PLANNING_GROUP);

  move_group.setPoseReferenceFrame(world_frame);
  move_group.setEndEffectorLink(ee_link);

  RCLCPP_INFO(logger, "Planning group: %s", PLANNING_GROUP.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  if (enable_wrist1_constraint) {
    const double center_rad = deg2rad(wrist1_center_deg);
    const double tol_rad    = deg2rad(wrist1_tolerance_deg);

    const auto constraints = make_wrist1_joint_constraint(
      wrist1_joint_name, center_rad, tol_rad);

    move_group.setPathConstraints(constraints);

    RCLCPP_INFO(logger,
      "Wrist1 PathConstraint ENABLED: joint=%s center=%.1fdeg (norm=%.3frad) tol=%.1fdeg (%.3frad)",
      wrist1_joint_name.c_str(),
      wrist1_center_deg, norm_angle(center_rad),
      wrist1_tolerance_deg, tol_rad);
  } else {
    move_group.clearPathConstraints();
    RCLCPP_INFO(logger, "Wrist1 PathConstraint DISABLED");
  }

  // ============================================================
  // RTAB-Map params client (app_node)
  // ============================================================
  std::shared_ptr<rclcpp::SyncParametersClient> rtabmap_params_client;
  if (switch_rtabmap) {
    rtabmap_params_client = std::make_shared<rclcpp::SyncParametersClient>(app_node, rtabmap_node_name);

    std::vector<rclcpp::Parameter> coarse_params;
    coarse_params.emplace_back("Rtabmap/DetectionRate", coarse_detection_rate);
    if (set_grid_params) {
      coarse_params.emplace_back("Grid/CellSize", coarse_grid_cell);
      coarse_params.emplace_back("Grid/RangeMax", coarse_range_max);
    }
    (void)set_rtabmap_params_best_effort(logger, rtabmap_params_client, coarse_params, wait_param_service_sec);
  }

  // ============================================================
  // Distance computation (camera-goal / camera-plant) via TF (app_node)
  // ============================================================
  auto compute_distances = [&](double &dist_goal, double &dist_plant) -> bool {
    geometry_msgs::msg::TransformStamped Twc, Twg, Twp;

    if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, ee_link, Twc, tf_timeout_sec))
      return false;

    if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Twg, tf_timeout_sec))
      return false;

    const bool has_plant = lookup_tf_blocking(tf_buffer, app_node, world_frame, plant_frame, Twp, 0.5);

    dist_goal = dist_xyz(Twc.transform.translation, Twg.transform.translation);
    dist_plant = has_plant
      ? dist_xyz(Twc.transform.translation, Twp.transform.translation)
      : std::numeric_limits<double>::infinity();

    return true;
  };

  // ============================================================
  // COARSE: pre approach
  // ============================================================
  RCLCPP_INFO(logger, "=== COARSE: pre approach (look-at %s) ===", leaf_center_frame.c_str());
  move_group.setPlanningTime(coarse_plan_time);
  move_group.setMaxVelocityScalingFactor(coarse_vel);
  move_group.setMaxAccelerationScalingFactor(coarse_acc);

  bool switched = false;

  for (int attempt = 1; attempt <= max_coarse_attempts && rclcpp::ok(); ++attempt) {
    geometry_msgs::msg::TransformStamped Tw_leaf;
    if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Tw_leaf, tf_timeout_sec)) {
      RCLCPP_ERROR(logger, "Cannot get TF to leaf center: %s", leaf_center_frame.c_str());
      break;
    }
    const geometry_msgs::msg::Vector3 leaf_pos_world = pos_from_tf(Tw_leaf);

    const auto cur_ps = move_group.getCurrentPose(ee_link);
    const geometry_msgs::msg::Vector3 cam_pos_world =
      v3(cur_ps.pose.position.x, cur_ps.pose.position.y, cur_ps.pose.position.z);

    const geometry_msgs::msg::Vector3 pre_pos_world =
      compute_leaf_pre_position(cam_pos_world, leaf_pos_world, pre_back_m, pre_down_m);

    {
      const double dist_cur_to_leaf = v3_norm(v3_sub(leaf_pos_world, cam_pos_world));
      const double dist_pre_to_leaf = v3_norm(v3_sub(leaf_pos_world, pre_pos_world));
      const double dist_cur_to_pre  = v3_norm(v3_sub(pre_pos_world, cam_pos_world));

      RCLCPP_INFO(logger,
        "[COARSE][attempt %d/%d] cur=(%.3f,%.3f,%.3f) leaf=(%.3f,%.3f,%.3f) pre=(%.3f,%.3f,%.3f) "
        "d(cur->leaf)=%.3f d(pre->leaf)=%.3f d(cur->pre)=%.3f",
        attempt, max_coarse_attempts,
        cam_pos_world.x, cam_pos_world.y, cam_pos_world.z,
        leaf_pos_world.x, leaf_pos_world.y, leaf_pos_world.z,
        pre_pos_world.x,  pre_pos_world.y,  pre_pos_world.z,
        dist_cur_to_leaf, dist_pre_to_leaf, dist_cur_to_pre);
    }

    RCLCPP_INFO(logger,
      "[COARSE] attempt %d/%d: move to PRE (back=%.3f down=%.3f) pos=(%.3f,%.3f,%.3f)",
      attempt, max_coarse_attempts, pre_back_m, pre_down_m,
      pre_pos_world.x, pre_pos_world.y, pre_pos_world.z);

    if (enable_distance_based_slowdown) {
      double dist_goal  = std::numeric_limits<double>::infinity();
      double dist_plant = std::numeric_limits<double>::infinity();
      if (compute_distances(dist_goal, dist_plant)) {
        apply_distance_based_scaling(
          logger, move_group, dist_goal,
          th_slow_m, th_very_slow_m,
          coarse_vel, coarse_acc,
          coarse_vel_slow, coarse_acc_slow,
          coarse_vel_vslow, coarse_acc_vslow);
      } else {
        move_group.setMaxVelocityScalingFactor(coarse_vel);
        move_group.setMaxAccelerationScalingFactor(coarse_acc);
      }
    } else {
      move_group.setMaxVelocityScalingFactor(coarse_vel);
      move_group.setMaxAccelerationScalingFactor(coarse_acc);
    }

    bool ok = false;

    if (enable_segmented_motion) {
      ok = move_segmented_lookat(
        app_node, logger, tf_buffer,
        move_group,
        world_frame, ee_link,
        leaf_center_frame,
        pre_pos_world,
        segment_max_m,
        arrive_pos_tol_m,
        &obstacle_monitor,
        enable_runtime_monitor,
        monitor_rate_hz,
        stop_dist_m,
        pc_stale_sec,
        max_runtime_replans,
        replan_backoff_sec,
        tf_timeout_sec,
        enable_goal_proximity_gating,
        dist_mid_m, dist_near_m,
        stop_dist_mid_m, stop_dist_near_m,
        enable_leaf_self_ignore,
        leaf_ignore_within_m,
        leaf_ignore_band_m,
        "COARSE", attempt, "pre",
        verbose_motion_log, monitor_log_every_n,
        log_on_stop_event,
        log_wrist1_angle,
        wrist1_joint_name,
        enable_late_ee_orientation_constraint,
        coarse_orient_from_seg,
        fine_orient_from_seg,
        orient_enable_rem_m,
        ee_orient_tol_x_rad,
        ee_orient_tol_y_rad,
        ee_orient_tol_z_rad,
        ee_orient_weight);
    } else {
      const geometry_msgs::msg::Vector3 world_up = v3(0,0,1);
      const auto goal_pose = make_lookat_pose(world_frame, leaf_pos_world, world_up, pre_pos_world);

      MotionCtx ctx;
      ctx.phase   = "COARSE";
      ctx.attempt = attempt;
      ctx.segment = -1;
      ctx.replan  = 0;
      ctx.tag     = "pre";

      ok = plan_execute_with_runtime_replan(
        app_node, logger,
        tf_buffer, world_frame, leaf_center_frame,
        move_group,
        goal_pose, ee_link,
        &obstacle_monitor,
        enable_runtime_monitor,
        monitor_rate_hz,
        stop_dist_m,
        pc_stale_sec,
        max_runtime_replans,
        replan_backoff_sec,
        enable_goal_proximity_gating,
        dist_mid_m, dist_near_m,
        stop_dist_mid_m, stop_dist_near_m,
        enable_leaf_self_ignore,
        leaf_ignore_within_m,
        leaf_ignore_band_m,
        ctx,
        verbose_motion_log,
        monitor_log_every_n,
        log_on_stop_event,
        log_wrist1_angle,
        wrist1_joint_name);
    }

    if (!ok) {
      RCLCPP_WARN(logger, "[COARSE] pre approach failed (attempt %d). Wait %.2fs and retry.",
                  attempt, coarse_retry_wait_sec);
      rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
      continue;
    }

    double dist_goal  = std::numeric_limits<double>::infinity();
    double dist_plant = std::numeric_limits<double>::infinity();

    if (!compute_distances(dist_goal, dist_plant)) {
      RCLCPP_WARN(logger, "Distance computation failed after coarse pre approach.");
      rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
      continue;
    }

    const bool cond_goal  = (dist_goal  < switch_goal_m);
    const bool cond_plant = (dist_plant < switch_plant_m);

    RCLCPP_INFO(logger,
      "[COARSE] dist(camera, leaf)=%.3f (thr=%.3f), dist(camera, plant)=%.3f (thr=%.3f)",
      dist_goal, switch_goal_m, dist_plant, switch_plant_m);

    if (cond_goal || cond_plant) {
      RCLCPP_INFO(logger, "[COARSE] switch met -> enter FINE. leaf=%s plant=%s",
                  cond_goal ? "true" : "false",
                  cond_plant ? "true" : "false");
      switched = true;
      break;
    }

    RCLCPP_INFO(logger, "[COARSE] switch NOT met yet. Retrying...");
    rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
  }

  if (!switched) {
    RCLCPP_ERROR(logger, "Failed to reach switch condition. Abort.");
    return cleanup_and_exit(1);
  }

  // ============================================================
  // Switch RTAB-Map params to FINE
  // ============================================================
  if (switch_rtabmap && rtabmap_params_client) {
    RCLCPP_INFO(logger, "Switching RTAB-Map parameters to FINE...");

    std::vector<rclcpp::Parameter> fine_params;
    fine_params.emplace_back("Rtabmap/DetectionRate", fine_detection_rate);
    if (set_grid_params) {
      fine_params.emplace_back("Grid/CellSize", fine_grid_cell);
      fine_params.emplace_back("Grid/RangeMax", fine_range_max);
    }

    (void)set_rtabmap_params_best_effort(logger, rtabmap_params_client, fine_params, wait_param_service_sec);
    rclcpp::sleep_for(sec_to_ns(fine_settle_wait_sec));
  }

  // ============================================================
  // FINE: approach shot pose (leaf_target_shot) while looking at leaf_center
  // ============================================================
  RCLCPP_INFO(logger, "=== FINE: go to %s (look-at %s) ===",
              fine_goal_frame.c_str(), leaf_center_frame.c_str());

  move_group.setPlanningTime(fine_plan_time);
  move_group.setMaxVelocityScalingFactor(fine_vel);
  move_group.setMaxAccelerationScalingFactor(fine_acc);

  geometry_msgs::msg::TransformStamped Tw_leaf, Tw_shot;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Tw_leaf, tf_timeout_sec)) {
    RCLCPP_ERROR(logger, "Cannot get TF to leaf center: %s", leaf_center_frame.c_str());
    return cleanup_and_exit(1);
  }
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, fine_goal_frame, Tw_shot, tf_timeout_sec)) {
    RCLCPP_ERROR(logger, "Cannot get TF to fine goal: %s", fine_goal_frame.c_str());
    return cleanup_and_exit(1);
  }

  const geometry_msgs::msg::Vector3 leaf_pos_world = pos_from_tf(Tw_leaf);
  const geometry_msgs::msg::Vector3 shot_pos_world = pos_from_tf(Tw_shot);
  const geometry_msgs::msg::Vector3 world_up = v3(0,0,1);

  bool fine_ok = false;

  if (enable_fine_cartesian) {
    for (int k = 0; rclcpp::ok(); ++k) {
      const auto shot_pose = make_lookat_pose(world_frame, leaf_pos_world, world_up, shot_pos_world);

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(1);
      waypoints.push_back(shot_pose.pose);

      moveit_msgs::msg::RobotTrajectory traj;
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        cart_eef_step,
        cart_jump_thr,
        traj,
        /*avoid_collisions*/ true);

      RCLCPP_INFO(logger, "CartesianPath fraction=%.3f (min=%.3f)", fraction, cart_min_fraction);

      if (fraction >= cart_min_fraction) {
        const bool exec_ok = execute_traj_with_monitor_gated(
          app_node, logger,
          tf_buffer, world_frame, leaf_center_frame, ee_link,
          move_group,
          traj,
          &obstacle_monitor,
          enable_runtime_monitor,
          monitor_rate_hz,
          stop_dist_m,
          pc_stale_sec,
          replan_backoff_sec,
          enable_goal_proximity_gating,
          dist_mid_m, dist_near_m,
          stop_dist_mid_m, stop_dist_near_m,
          enable_leaf_self_ignore,
          leaf_ignore_within_m,
          leaf_ignore_band_m);

        if (exec_ok) {
          fine_ok = true;
          break;
        }

        if (k >= max_runtime_replans) {
          RCLCPP_ERROR(logger, "Cartesian retries exceeded (%d).", max_runtime_replans);
          fine_ok = false;
          break;
        }

        RCLCPP_WARN(logger, "Cartesian interrupted; recompute from current state (retry %d/%d).",
                    (k+1), max_runtime_replans);
        continue;
      } else {
        RCLCPP_WARN(logger, "Cartesian fraction too low; fallback to normal planning.");
        break;
      }
    }
  }

  if (!fine_ok) {
    if (enable_segmented_motion) {
      fine_ok = move_segmented_lookat(
        app_node, logger, tf_buffer,
        move_group,
        world_frame, ee_link,
        leaf_center_frame,
        shot_pos_world,
        segment_max_m,
        arrive_pos_tol_m,
        &obstacle_monitor,
        enable_runtime_monitor,
        monitor_rate_hz,
        stop_dist_m,
        pc_stale_sec,
        max_runtime_replans,
        replan_backoff_sec,
        tf_timeout_sec,
        enable_goal_proximity_gating,
        dist_mid_m, dist_near_m,
        stop_dist_mid_m, stop_dist_near_m,
        enable_leaf_self_ignore,
        leaf_ignore_within_m,
        leaf_ignore_band_m,
        "FINE", -1, "shot",
        verbose_motion_log, monitor_log_every_n,
        log_on_stop_event,
        log_wrist1_angle,
        wrist1_joint_name,
        enable_late_ee_orientation_constraint,
        coarse_orient_from_seg,
        fine_orient_from_seg,
        orient_enable_rem_m,
        ee_orient_tol_x_rad,
        ee_orient_tol_y_rad,
        ee_orient_tol_z_rad,
        ee_orient_weight);
    } else {
      const auto shot_pose = make_lookat_pose(world_frame, leaf_pos_world, world_up, shot_pos_world);

      MotionCtx ctx;
      ctx.phase   = "FINE";
      ctx.attempt = -1;
      ctx.segment = -1;
      ctx.replan  = 0;
      ctx.tag     = "shot";

      fine_ok = plan_execute_with_runtime_replan(
        app_node, logger,
        tf_buffer, world_frame, leaf_center_frame,
        move_group,
        shot_pose, ee_link,
        &obstacle_monitor,
        enable_runtime_monitor,
        monitor_rate_hz,
        stop_dist_m,
        pc_stale_sec,
        max_runtime_replans,
        replan_backoff_sec,
        enable_goal_proximity_gating,
        dist_mid_m, dist_near_m,
        stop_dist_mid_m, stop_dist_near_m,
        enable_leaf_self_ignore,
        leaf_ignore_within_m,
        leaf_ignore_band_m,
        ctx,
        verbose_motion_log,
        monitor_log_every_n,
        log_on_stop_event,
        log_wrist1_angle,
        wrist1_joint_name);
    }
  }

  if (!fine_ok) {
    RCLCPP_ERROR(logger, "FINE approach failed.");
    return cleanup_and_exit(1);
  }

  // ============================================================
  // VIEW: rotate in place to leaf_target_view orientation
  // ============================================================
  RCLCPP_INFO(logger, "=== VIEW: align orientation to %s (position fixed) ===",
              view_goal_frame.c_str());

  move_group.setPlanningTime(view_plan_time);
  move_group.setMaxVelocityScalingFactor(view_vel);
  move_group.setMaxAccelerationScalingFactor(view_acc);

  geometry_msgs::msg::TransformStamped Tw_view;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, view_goal_frame, Tw_view, tf_timeout_sec)) {
    RCLCPP_ERROR(logger, "Cannot get TF to view goal: %s", view_goal_frame.c_str());
    return cleanup_and_exit(1);
  }

  const auto cur_ps2 = move_group.getCurrentPose(ee_link);

  geometry_msgs::msg::PoseStamped view_goal;
  view_goal.header.frame_id = world_frame;
  view_goal.pose.position = cur_ps2.pose.position;
  view_goal.pose.orientation = Tw_view.transform.rotation;

  MotionCtx vctx;
  vctx.phase   = "VIEW";
  vctx.attempt = -1;
  vctx.segment = -1;
  vctx.replan  = 0;
  vctx.tag     = "turn";

  const bool view_ok = plan_execute_with_runtime_replan(
    app_node, logger,
    tf_buffer, world_frame, leaf_center_frame,
    move_group,
    view_goal, ee_link,
    &obstacle_monitor,
    enable_runtime_monitor,
    monitor_rate_hz,
    stop_dist_m,
    pc_stale_sec,
    max_runtime_replans,
    replan_backoff_sec,
    enable_goal_proximity_gating,
    dist_mid_m, dist_near_m,
    stop_dist_mid_m, stop_dist_near_m,
    enable_leaf_self_ignore,
    leaf_ignore_within_m,
    leaf_ignore_band_m,
    vctx,
    verbose_motion_log,
    monitor_log_every_n,
    log_on_stop_event,
    log_wrist1_angle,
    wrist1_joint_name);

  if (!view_ok) {
    RCLCPP_ERROR(logger, "VIEW head turn failed.");
    return cleanup_and_exit(1);
  }

  RCLCPP_INFO(logger, "DONE: coarse(pre/look-at) -> fine(shot) -> view(turn) finished.");
  return cleanup_and_exit(0);
}
