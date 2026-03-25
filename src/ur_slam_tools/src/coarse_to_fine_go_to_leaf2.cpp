#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
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
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sec));
}

static bool is_finite3(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// TF: lookupTransform(target, source) returns target_T_source
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

    // TransformListener(spin_thread=false) のため、ここで回して詰まりを防ぐ
    rclcpp::spin_some(node);

    if ((clock->now() - start).seconds() > timeout_sec) {
      return false;
    }
    rate.sleep();
  }
  return false;
}

static geometry_msgs::msg::PoseStamped pose_from_tf(const std::string &frame_id,
                                                    const geometry_msgs::msg::TransformStamped &T)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame_id;
  p.pose.position.x = T.transform.translation.x;
  p.pose.position.y = T.transform.translation.y;
  p.pose.position.z = T.transform.translation.z;
  p.pose.orientation = T.transform.rotation;
  return p;
}

// Best-effort RTAB-Map parameter switching.
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
// - 近距離障害物を「カメラ前方（optical frame +Z）」で検知
// - MoveIt PlanningScene に障害物が入っていなくても “止まれる” ので衝突連鎖を断てる
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
    // センサ系 QoS（Gazebo/RealSense 系で安定しやすい）
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
    // PointCloud2 の field を探す（x,y,z を float32 想定）
    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto& f : msg->fields) {
      if (f.name == "x") off_x = static_cast<int>(f.offset);
      else if (f.name == "y") off_y = static_cast<int>(f.offset);
      else if (f.name == "z") off_z = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) {
      return;
    }

    // 近距離判定：optical frame の +Z が前方
    // ROI: |x|<roi_x, |y|<roi_y, 0<z<max_z
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

    // stamp は sim_time を使ってるので ROS time と合わせる
    double t = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
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
// Speed scaling helper（距離で段階的に落とす）
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

  v = std::max(0.0, std::min(1.0, v));
  a = std::max(0.0, std::min(1.0, a));

  move_group.setMaxVelocityScalingFactor(v);
  move_group.setMaxAccelerationScalingFactor(a);

  RCLCPP_INFO(logger,
              "Speed scaling by distance: dist=%.3f[m], vel=%.3f, acc=%.3f",
              dist_m, v, a);
}

// ============================================================
// Plan + Execute with monitoring (Aの中核)
// - execute を別スレッドで回し、メインで点群監視→危険なら stop→再計画
// ============================================================
static bool plan_execute_with_runtime_replan(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Logger& logger,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::string& ee_link_for_target,
    ForwardObstacleMonitor* obstacle_monitor,   // nullptr可
    bool enable_runtime_monitor,
    double monitor_rate_hz,
    double stop_dist_m,
    double pc_stale_sec,
    int max_runtime_replans,
    double replan_backoff_sec)
{
  int replans = 0;

  while (rclcpp::ok()) {
    // ---- planning ----
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal, ee_link_for_target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(logger, "Planning failed (EE=%s, goal_frame=%s).",
                   ee_link_for_target.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    // ---- execution in another thread ----
    std::atomic<bool> exec_done{false};
    std::atomic<int>  exec_code_val{0};   // MoveItErrorCode::val を保存
    std::atomic<bool> stop_requested{false};

    std::thread exec_thread([&](){
      RCLCPP_INFO(logger, "Executing plan...");
      auto ret = move_group.execute(plan);
      exec_code_val.store(ret.val);
      exec_done.store(true);
    });

    // ---- monitoring loop (main thread) ----
    rclcpp::Rate rate(std::max(1.0, monitor_rate_hz));
    const auto clock = node->get_clock();

    while (rclcpp::ok() && !exec_done.load()) {
      // コールバックを回す（点群/TF/param など）
      rclcpp::spin_some(node);

      if (enable_runtime_monitor && obstacle_monitor) {
        // 点群の鮮度確認
        const double now = clock->now().seconds();
        const double pc_stamp = obstacle_monitor->last_stamp_sec();
        const double min_forward = obstacle_monitor->last_min_forward_m();

        const bool stale = (pc_stamp <= 0.0) || ((now - pc_stamp) > pc_stale_sec);

        if (!stale && std::isfinite(min_forward) && (min_forward < stop_dist_m)) {
          // 危険：即停止 → 再計画へ
          RCLCPP_WARN(logger,
                      "Obstacle too close in camera forward ROI: min_z=%.3f[m] < %.3f[m]. STOP & REPLAN.",
                      min_forward, stop_dist_m);

          stop_requested.store(true);
          move_group.stop();  // まず MoveIt 側に停止要求
          break;
        }
      }

      rate.sleep();
    }

    // 停止要求した場合、execute が返るのを待つ
    if (exec_thread.joinable()) exec_thread.join();

    const int code = exec_code_val.load();

    // ---- outcome handling ----
    if (stop_requested.load()) {
      // runtime replan を続ける
      replans++;

      if (replans > max_runtime_replans) {
        RCLCPP_ERROR(logger, "Exceeded max runtime replans (%d). Give up this goal.", max_runtime_replans);
        return false;
      }

      // Gazebo / controller が落ち着くまで少し待つ（重要）
      rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
      continue;
    }

    // stopしていないのに失敗（例：state tolerance violation 等）
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution failed (MoveItErrorCode=%d).", code);

      // ここでも “現在状態からやり直し” を少回数だけ試す（衝突/追従乱れの回復狙い）
      replans++;
      if (replans > max_runtime_replans) {
        RCLCPP_ERROR(logger, "Exceeded max retries after execution failure (%d).", max_runtime_replans);
        return false;
      }

      RCLCPP_WARN(logger, "Retry by replanning from current state (retry %d/%d).",
                  replans, max_runtime_replans);
      rclcpp::sleep_for(sec_to_ns(replan_backoff_sec));
      continue;
    }

    // 成功
    RCLCPP_INFO(logger, "Execution SUCCESS.");
    return true;
  }

  return false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coarse_to_fine_go_to_leaf");
  const auto logger = node->get_logger();
  const auto clock  = node->get_clock();

  // use_sim_time（CLI/launch を尊重）
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter<bool>("use_sim_time", true);
  }
  bool use_sim_time = true;
  node->get_parameter("use_sim_time", use_sim_time);
  RCLCPP_INFO(logger, "use_sim_time=%s", use_sim_time ? "true" : "false");

  // -------------------------
  // Parameters
  // -------------------------
  // TF frames
  const std::string world_frame  = node->declare_parameter<std::string>("world_frame", "base_link");
  const std::string camera_frame = node->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  const std::string plant_frame  = node->declare_parameter<std::string>("plant_frame", "plant_base");

  // Goal frames
  const std::string coarse_goal_frame = node->declare_parameter<std::string>("coarse_goal_frame", "leaf_target_view");
  const std::string fine_goal_frame   = node->declare_parameter<std::string>("fine_goal_frame",   "leaf_target_shot");

  // IK solve link
  const std::string ee_link = node->declare_parameter<std::string>("ee_link", "camera_color_optical_frame");

  // Switch criteria (OR)
  const double switch_goal_m  = node->declare_parameter<double>("switch_goal_m",  0.25);
  const double switch_plant_m = node->declare_parameter<double>("switch_plant_m", 0.35);

  const int    max_coarse_attempts = node->declare_parameter<int>("max_coarse_attempts", 3);
  const double tf_timeout_sec      = node->declare_parameter<double>("tf_timeout_sec", 10.0);

  // Planning time
  const double coarse_plan_time = node->declare_parameter<double>("coarse_planning_time", 2.0);
  const double fine_plan_time   = node->declare_parameter<double>("fine_planning_time",   5.0);

  // Base speed scaling（まずはかなり遅く）
  const double coarse_vel = node->declare_parameter<double>("coarse_vel_scale", 0.02);
  const double coarse_acc = node->declare_parameter<double>("coarse_acc_scale", 0.02);
  const double fine_vel   = node->declare_parameter<double>("fine_vel_scale",   0.10);
  const double fine_acc   = node->declare_parameter<double>("fine_acc_scale",   0.10);

  // Distance-based slowdown (COARSE)
  const bool enable_distance_based_slowdown =
      node->declare_parameter<bool>("enable_distance_based_slowdown", true);

  const double th_slow_m       = node->declare_parameter<double>("slowdown_th_slow_m", 0.50);
  const double th_very_slow_m  = node->declare_parameter<double>("slowdown_th_very_slow_m", 0.30);

  const double coarse_vel_slow  = node->declare_parameter<double>("coarse_vel_scale_slow", 0.015);
  const double coarse_acc_slow  = node->declare_parameter<double>("coarse_acc_scale_slow", 0.015);
  const double coarse_vel_vslow = node->declare_parameter<double>("coarse_vel_scale_very_slow", 0.008);
  const double coarse_acc_vslow = node->declare_parameter<double>("coarse_acc_scale_very_slow", 0.008);

  const double coarse_retry_wait_sec =
      node->declare_parameter<double>("coarse_retry_wait_sec", 0.5);

  // ---- A: runtime monitor parameters ----
  const bool enable_runtime_monitor =
      node->declare_parameter<bool>("enable_runtime_monitor", true);

  const std::string pc_topic =
      node->declare_parameter<std::string>("pointcloud_topic", "/camera/depth/points");

  const double monitor_rate_hz =
      node->declare_parameter<double>("monitor_rate_hz", 30.0);

  // カメラ前方ROIで、これ未満の点が出たら停止
  const double stop_dist_m =
      node->declare_parameter<double>("stop_dist_m", 0.35);

  // ROI（光学座標で |x|,|y| を絞る＝視線中心付近だけを見る）
  const double roi_x_m =
      node->declare_parameter<double>("pc_roi_x_m", 0.18);
  const double roi_y_m =
      node->declare_parameter<double>("pc_roi_y_m", 0.18);

  // z の上限（遠い点は無視）
  const double pc_max_z_m =
      node->declare_parameter<double>("pc_max_z_m", 1.50);

  // 点群が古い場合は止めない（stale を誤検知停止しないため）
  const double pc_stale_sec =
      node->declare_parameter<double>("pc_stale_sec", 0.20);

  // 実行中の stop→replan を何回まで許すか（COARSE/FINE 共通で利用）
  const int max_runtime_replans =
      node->declare_parameter<int>("max_runtime_replans", 5);

  // stop/replan の合間に待つ（Gazebo物理・controllerが落ち着くのを待つ）
  const double replan_backoff_sec =
      node->declare_parameter<double>("replan_backoff_sec", 0.30);

  // RTAB-Map param switching
  const bool   switch_rtabmap         = node->declare_parameter<bool>("switch_rtabmap", true);
  const std::string rtabmap_node_name = node->declare_parameter<std::string>("rtabmap_node_name", "/rtabmap");
  const double wait_param_service_sec = node->declare_parameter<double>("wait_param_service_sec", 5.0);
  const double fine_settle_wait_sec   = node->declare_parameter<double>("fine_settle_wait_sec", 2.0);

  const double coarse_detection_rate = node->declare_parameter<double>("coarse_detection_rate", 2.0);
  const double fine_detection_rate   = node->declare_parameter<double>("fine_detection_rate", 10.0);

  const bool set_grid_params      = node->declare_parameter<bool>("set_grid_params", false);
  const double coarse_grid_cell   = node->declare_parameter<double>("coarse_grid_cell", 0.05);
  const double fine_grid_cell     = node->declare_parameter<double>("fine_grid_cell",   0.02);
  const double coarse_range_max   = node->declare_parameter<double>("coarse_range_max", 3.0);
  const double fine_range_max     = node->declare_parameter<double>("fine_range_max",   1.5);

  // -------------------------
  // TF
  // -------------------------
  tf2_ros::Buffer tf_buffer(clock);
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  // -------------------------
  // Point cloud obstacle monitor
  // -------------------------
  ForwardObstacleMonitor obstacle_monitor(node, pc_topic, roi_x_m, roi_y_m, pc_max_z_m);

  RCLCPP_INFO(logger, "Runtime monitor: %s", enable_runtime_monitor ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(logger, "  pc_topic=%s stop_dist=%.3f roi_x=%.3f roi_y=%.3f max_z=%.3f stale=%.3f rate=%.1f",
              pc_topic.c_str(), stop_dist_m, roi_x_m, roi_y_m, pc_max_z_m, pc_stale_sec, monitor_rate_hz);

  // -------------------------
  // MoveIt
  // -------------------------
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPoseReferenceFrame(world_frame);
  move_group.setEndEffectorLink(ee_link);

  move_group.setPlanningTime(coarse_plan_time);
  move_group.setMaxVelocityScalingFactor(coarse_vel);
  move_group.setMaxAccelerationScalingFactor(coarse_acc);

  RCLCPP_INFO(logger, "Planning group: %s", PLANNING_GROUP.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // -------------------------
  // RTAB-Map params client
  // -------------------------
  std::shared_ptr<rclcpp::SyncParametersClient> rtabmap_params_client;
  if (switch_rtabmap) {
    rtabmap_params_client = std::make_shared<rclcpp::SyncParametersClient>(node, rtabmap_node_name);

    std::vector<rclcpp::Parameter> coarse_params;
    coarse_params.emplace_back("Rtabmap/DetectionRate", coarse_detection_rate);
    if (set_grid_params) {
      coarse_params.emplace_back("Grid/CellSize", coarse_grid_cell);
      coarse_params.emplace_back("Grid/RangeMax", coarse_range_max);
    }
    (void)set_rtabmap_params_best_effort(logger, rtabmap_params_client, coarse_params, wait_param_service_sec);
  }

  // -------------------------
  // Distance computation (camera-goal / camera-plant)
  // -------------------------
  auto compute_distances = [&](double &dist_goal, double &dist_plant) -> bool {
    geometry_msgs::msg::TransformStamped Twc, Twg, Twp;

    if (!lookup_tf_blocking(tf_buffer, node, world_frame, camera_frame, Twc, tf_timeout_sec))
      return false;

    if (!lookup_tf_blocking(tf_buffer, node, world_frame, coarse_goal_frame, Twg, tf_timeout_sec))
      return false;

    const bool has_plant = lookup_tf_blocking(tf_buffer, node, world_frame, plant_frame, Twp, 0.5);

    dist_goal = dist_xyz(Twc.transform.translation, Twg.transform.translation);
    dist_plant = has_plant
      ? dist_xyz(Twc.transform.translation, Twp.transform.translation)
      : std::numeric_limits<double>::infinity();

    return true;
  };

  // ============================================================
  // COARSE phase
  // ============================================================
  RCLCPP_INFO(logger, "=== COARSE phase: target=%s ===", coarse_goal_frame.c_str());
  move_group.setPlanningTime(coarse_plan_time);
  move_group.setMaxVelocityScalingFactor(coarse_vel);
  move_group.setMaxAccelerationScalingFactor(coarse_acc);

  bool switched = false;

  for (int attempt = 1; attempt <= max_coarse_attempts && rclcpp::ok(); ++attempt) {
    geometry_msgs::msg::TransformStamped Twg;

    if (!lookup_tf_blocking(tf_buffer, node, world_frame, coarse_goal_frame, Twg, tf_timeout_sec)) {
      RCLCPP_ERROR(logger, "Cannot get TF to coarse goal: %s", coarse_goal_frame.c_str());
      break;
    }
    const auto coarse_goal_pose = pose_from_tf(world_frame, Twg);

    // 近づいたらさらに減速（衝突・暴れ対策）
    if (enable_distance_based_slowdown) {
      double dist_goal  = std::numeric_limits<double>::infinity();
      double dist_plant = std::numeric_limits<double>::infinity();

      if (compute_distances(dist_goal, dist_plant)) {
        apply_distance_based_scaling(
            logger, move_group, dist_goal,
            th_slow_m, th_very_slow_m,
            /*nominal*/ coarse_vel,        coarse_acc,
            /*slow*/    coarse_vel_slow,   coarse_acc_slow,
            /*vslow*/   coarse_vel_vslow,  coarse_acc_vslow);
      } else {
        move_group.setMaxVelocityScalingFactor(coarse_vel);
        move_group.setMaxAccelerationScalingFactor(coarse_acc);
      }
    }

    RCLCPP_INFO(logger, "[COARSE] attempt %d/%d: plan->execute(with monitor)", attempt, max_coarse_attempts);

    const bool ok = plan_execute_with_runtime_replan(
      node, logger, move_group,
      coarse_goal_pose, ee_link,
      &obstacle_monitor,
      enable_runtime_monitor,
      monitor_rate_hz,
      stop_dist_m,
      pc_stale_sec,
      max_runtime_replans,
      replan_backoff_sec);

    if (!ok) {
      RCLCPP_WARN(logger, "[COARSE] failed to reach coarse goal (attempt %d). Wait %.2fs and retry.",
                  attempt, coarse_retry_wait_sec);
      rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
      continue;
    }

    // 実行後の距離評価（スイッチ条件）
    double dist_goal  = std::numeric_limits<double>::infinity();
    double dist_plant = std::numeric_limits<double>::infinity();

    if (!compute_distances(dist_goal, dist_plant)) {
      RCLCPP_WARN(logger, "Distance computation failed after coarse execute.");
      rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
      continue;
    }

    const bool cond_goal  = (dist_goal  < switch_goal_m);
    const bool cond_plant = (dist_plant < switch_plant_m);

    RCLCPP_INFO(logger,
                "[COARSE] dist(camera, goal)=%.3f (thr=%.3f), dist(camera, plant)=%.3f (thr=%.3f)",
                dist_goal, switch_goal_m, dist_plant, switch_plant_m);

    if (cond_goal || cond_plant) {
      RCLCPP_INFO(logger, "[COARSE] switch met (enter FINE). goal=%s plant=%s",
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
    rclcpp::shutdown();
    return 1;
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
  // FINE phase
  // ============================================================
  RCLCPP_INFO(logger, "=== FINE phase: target=%s ===", fine_goal_frame.c_str());
  move_group.setPlanningTime(fine_plan_time);
  move_group.setMaxVelocityScalingFactor(fine_vel);
  move_group.setMaxAccelerationScalingFactor(fine_acc);

  geometry_msgs::msg::TransformStamped Twf;
  if (!lookup_tf_blocking(tf_buffer, node, world_frame, fine_goal_frame, Twf, tf_timeout_sec)) {
    RCLCPP_ERROR(logger, "Cannot get TF to fine goal: %s", fine_goal_frame.c_str());
    rclcpp::shutdown();
    return 1;
  }
  const auto fine_goal_pose = pose_from_tf(world_frame, Twf);

  // FINE でも同じ監視で “最終接近での衝突” を避ける
  const bool fine_ok = plan_execute_with_runtime_replan(
    node, logger, move_group,
    fine_goal_pose, ee_link,
    &obstacle_monitor,
    enable_runtime_monitor,
    monitor_rate_hz,
    // 微動のほうが安全マージンを大きくして止めたいなら stop_dist を上げてもよい
    stop_dist_m,
    pc_stale_sec,
    max_runtime_replans,
    replan_backoff_sec);

  if (!fine_ok) {
    RCLCPP_ERROR(logger, "FINE execute failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "DONE: coarse-to-fine go_to_leaf finished.");
  rclcpp::shutdown();
  return 0;
}
