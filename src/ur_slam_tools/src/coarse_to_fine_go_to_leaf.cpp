#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <chrono>

// ------------------------------
// Small helpers
// ------------------------------
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

// TF: lookupTransform(target, source) returns transform that transforms data from source into target
// i.e. target_T_source
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

    // 重要：
    // TransformListener(spin_thread=false) で作っているため、
    // このノードのコールバックを定期的に回して TF 受信を詰まらせない。
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

static bool plan_and_execute_pose(
    const rclcpp::Logger &logger,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::PoseStamped &goal,
    const std::string &ee_link_for_target)
{
  // デバッグ性向上：
  // 前回のターゲットが残っていると挙動追跡が難しいため、毎回クリアする。
  move_group.clearPoseTargets();

  // 重要：プラン直前に最新の現在状態を start state に反映
  // （Gazebo / ros2_control ではこれを怠ると古い状態でプランしやすい）
  move_group.setStartStateToCurrentState();

  // 指定した EE リンク（例：camera_color_optical_frame）に対して IK を解く
  move_group.setPoseTarget(goal, ee_link_for_target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!planned) {
    RCLCPP_ERROR(logger, "Planning failed (EE=%s, goal_frame=%s).",
                 ee_link_for_target.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Executing plan (EE=%s)...", ee_link_for_target.c_str());
  const auto exec_ret = move_group.execute(plan);
  if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed (MoveItErrorCode=%d).", exec_ret.val);
    return false;
  }
  return true;
}

// ------------------------------
// 速度を段階的に落とすためのヘルパ
// ------------------------------
// 「近いほど遅く」したいとき、距離に応じてスケーリングを変更する。
// - dist_m が閾値1以下 → slow スケール
// - dist_m が閾値2以下 → very_slow スケール
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
  // デフォルトは nominal
  double v = v_nominal;
  double a = a_nominal;

  if (dist_m < th_very_slow_m) {
    v = v_vslow;
    a = a_vslow;
  } else if (dist_m < th_slow_m) {
    v = v_slow;
    a = a_slow;
  }

  // setMax*ScalingFactor は [0,1] 想定。念のためクランプ。
  v = std::max(0.0, std::min(1.0, v));
  a = std::max(0.0, std::min(1.0, a));

  move_group.setMaxVelocityScalingFactor(v);
  move_group.setMaxAccelerationScalingFactor(a);

  RCLCPP_INFO(logger,
              "Speed scaling applied by distance: dist=%.3f[m], vel=%.3f, acc=%.3f",
              dist_m, v, a);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coarse_to_fine_go_to_leaf");
  const auto logger = node->get_logger();
  const auto clock  = node->get_clock();

  // use_sim_time: launch/CLIから与えられることが多いので二重declareを避ける
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
  const std::string world_frame      = node->declare_parameter<std::string>("world_frame", "base_link");
  const std::string camera_frame     = node->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  const std::string plant_frame      = node->declare_parameter<std::string>("plant_frame", "plant_base");

  // Goal frames
  const std::string coarse_goal_frame = node->declare_parameter<std::string>("coarse_goal_frame", "leaf_target_view");
  const std::string fine_goal_frame   = node->declare_parameter<std::string>("fine_goal_frame",   "leaf_target_shot");

  // IK を解くリンク（カメラを EE として合わせたい場合は camera_color_optical_frame など）
  const std::string ee_link           = node->declare_parameter<std::string>("ee_link", "camera_color_optical_frame");

  // Switch criteria (OR condition)
  const double switch_goal_m  = node->declare_parameter<double>("switch_goal_m",  0.25);
  const double switch_plant_m = node->declare_parameter<double>("switch_plant_m", 0.35);

  const int    max_coarse_attempts = node->declare_parameter<int>("max_coarse_attempts", 3);
  const double tf_timeout_sec      = node->declare_parameter<double>("tf_timeout_sec", 10.0);

  // MoveIt planning time
  const double coarse_plan_time = node->declare_parameter<double>("coarse_planning_time", 2.0);
  const double fine_plan_time   = node->declare_parameter<double>("fine_planning_time",   5.0);

  // ------------------------------------------
  // 速度スケーリング（ここが「ゆっくり」にする主パラメータ）
  // ------------------------------------------
  // まずは “粗動も遅く” したいとのことなので、デフォルトをかなり低めにしてあります。
  // テストが進んだら 0.02 → 0.05 → 0.1 のように段階的に上げてください。
  const double coarse_vel = node->declare_parameter<double>("coarse_vel_scale", 0.05);
  const double coarse_acc = node->declare_parameter<double>("coarse_acc_scale", 0.05);

  const double fine_vel   = node->declare_parameter<double>("fine_vel_scale",   0.10);
  const double fine_acc   = node->declare_parameter<double>("fine_acc_scale",   0.10);

  // 追加：粗動中に「ターゲットに近づいたらさらに減速」するための段階減速パラメータ
  // （衝突リスクが上がる距離帯だけ、より安全側に振る）
  const bool enable_distance_based_slowdown =
      node->declare_parameter<bool>("enable_distance_based_slowdown", true);

  // dist < th_slow で slow、dist < th_very_slow で very_slow
  const double th_slow_m       = node->declare_parameter<double>("slowdown_th_slow_m", 0.50);
  const double th_very_slow_m  = node->declare_parameter<double>("slowdown_th_very_slow_m", 0.30);

  // slow/very_slow のスケーリング
  const double coarse_vel_slow  = node->declare_parameter<double>("coarse_vel_scale_slow", 0.03);
  const double coarse_acc_slow  = node->declare_parameter<double>("coarse_acc_scale_slow", 0.03);
  const double coarse_vel_vslow = node->declare_parameter<double>("coarse_vel_scale_very_slow", 0.015);
  const double coarse_acc_vslow = node->declare_parameter<double>("coarse_acc_scale_very_slow", 0.015);

  // 追加：粗動ループの各試行の間に「待つ」ことで挙動を落ち着かせたい場合に使う
  // （Gazebo の物理が落ち着く、TF/状態更新が追いつくなどのメリット）
  const double coarse_retry_wait_sec =
      node->declare_parameter<double>("coarse_retry_wait_sec", 0.5);

  // RTAB-Map param switching
  const bool   switch_rtabmap         = node->declare_parameter<bool>("switch_rtabmap", true);
  const std::string rtabmap_node_name = node->declare_parameter<std::string>("rtabmap_node_name", "/rtabmap");
  const double wait_param_service_sec = node->declare_parameter<double>("wait_param_service_sec", 5.0);
  const double fine_settle_wait_sec   = node->declare_parameter<double>("fine_settle_wait_sec", 2.0);

  const double coarse_detection_rate = node->declare_parameter<double>("coarse_detection_rate", 2.0);
  const double fine_detection_rate   = node->declare_parameter<double>("fine_detection_rate", 10.0);

  // Optional grid parameters (type mismatch が起きやすいのでデフォルト false 推奨)
  const bool set_grid_params      = node->declare_parameter<bool>("set_grid_params", false);
  const double coarse_grid_cell   = node->declare_parameter<double>("coarse_grid_cell", 0.05);
  const double fine_grid_cell     = node->declare_parameter<double>("fine_grid_cell",   0.02);
  const double coarse_range_max   = node->declare_parameter<double>("coarse_range_max", 3.0);
  const double fine_range_max     = node->declare_parameter<double>("fine_range_max",   1.5);

  // -------------------------
  // TF
  // -------------------------
  tf2_ros::Buffer tf_buffer(clock);

  // 推奨：
  // TransformListener を node に紐付け、spin_thread=false にする。
  // → その代わり lookup_tf_blocking 内で spin_some を呼び、受信を回す。
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  // -------------------------
  // MoveIt
  // -------------------------
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Pose target を解く座標系（通常は "world" 推奨。TF ツリーのルートと合わせる）
  move_group.setPoseReferenceFrame(world_frame);

  // ここで EE link を固定しておく（setPoseTarget の第二引数とも整合させる）
  move_group.setEndEffectorLink(ee_link);

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

    // 粗動モード：DetectionRate を落として計算負荷を下げる等
    std::vector<rclcpp::Parameter> coarse_params;
    coarse_params.emplace_back("Rtabmap/DetectionRate", coarse_detection_rate);
    if (set_grid_params) {
      coarse_params.emplace_back("Grid/CellSize", coarse_grid_cell);
      coarse_params.emplace_back("Grid/RangeMax", coarse_range_max);
    }
    (void)set_rtabmap_params_best_effort(logger, rtabmap_params_client, coarse_params, wait_param_service_sec);
  }

  // -------------------------
  // Distance computation
  // -------------------------
  // world_T_camera / world_T_goal / world_T_plant を TF から取り、距離を計算する
  auto compute_distances = [&](double &dist_goal, double &dist_plant) -> bool {
    geometry_msgs::msg::TransformStamped Twc, Twg, Twp;

    // world_T_camera
    if (!lookup_tf_blocking(tf_buffer, node, world_frame, camera_frame, Twc, tf_timeout_sec))
      return false;

    // world_T_coarse_goal
    if (!lookup_tf_blocking(tf_buffer, node, world_frame, coarse_goal_frame, Twg, tf_timeout_sec))
      return false;

    // plant は存在しない場合もあるので短いタイムアウトで best-effort
    const bool has_plant = lookup_tf_blocking(tf_buffer, node, world_frame, plant_frame, Twp, 0.5);

    dist_goal = dist_xyz(Twc.transform.translation, Twg.transform.translation);
    dist_plant = has_plant
      ? dist_xyz(Twc.transform.translation, Twp.transform.translation)
      : std::numeric_limits<double>::infinity();

    return true;
  };

  // -------------------------
  // COARSE phase
  // -------------------------
  RCLCPP_INFO(logger, "=== COARSE phase: target=%s ===", coarse_goal_frame.c_str());

  // 粗動のプランニング時間とベース速度を設定
  move_group.setPlanningTime(coarse_plan_time);
  move_group.setMaxVelocityScalingFactor(coarse_vel);
  move_group.setMaxAccelerationScalingFactor(coarse_acc);

  bool switched = false;

  for (int attempt = 1; attempt <= max_coarse_attempts && rclcpp::ok(); ++attempt) {
    geometry_msgs::msg::TransformStamped Twg;

    // world_T_goal を取得（目標は常に TF から最新を取る）
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
        RCLCPP_WARN(logger, "Distance computation failed; keep nominal coarse scaling.");
        move_group.setMaxVelocityScalingFactor(coarse_vel);
        move_group.setMaxAccelerationScalingFactor(coarse_acc);
      }
    }

    RCLCPP_INFO(logger, "[COARSE] attempt %d/%d: plan->execute", attempt, max_coarse_attempts);

    // 重要：execute の失敗は無視せず、次の試行前に少し待って状態を落ち着かせる
    const bool ok = plan_and_execute_pose(logger, move_group, coarse_goal_pose, ee_link);
    if (!ok) {
      RCLCPP_WARN(logger, "[COARSE] plan/execute failed; wait %.2fs and retry.", coarse_retry_wait_sec);
      rclcpp::sleep_for(sec_to_ns(coarse_retry_wait_sec));
      continue;
    }

    // 実行後の距離評価（スイッチ条件）
    double dist_goal  = std::numeric_limits<double>::infinity();
    double dist_plant = std::numeric_limits<double>::infinity();

    if (!compute_distances(dist_goal, dist_plant)) {
      RCLCPP_WARN(logger, "Distance computation failed.");
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

  // -------------------------
  // Switch RTAB-Map params to FINE
  // -------------------------
  if (switch_rtabmap && rtabmap_params_client) {
    RCLCPP_INFO(logger, "Switching RTAB-Map parameters to FINE...");

    std::vector<rclcpp::Parameter> fine_params;
    fine_params.emplace_back("Rtabmap/DetectionRate", fine_detection_rate);
    if (set_grid_params) {
      fine_params.emplace_back("Grid/CellSize", fine_grid_cell);
      fine_params.emplace_back("Grid/RangeMax", fine_range_max);
    }

    (void)set_rtabmap_params_best_effort(logger, rtabmap_params_client, fine_params, wait_param_service_sec);

    // パラメータ切替直後に地図/検出が落ち着くのを待つ
    rclcpp::sleep_for(sec_to_ns(fine_settle_wait_sec));
  }

  // -------------------------
  // FINE phase
  // -------------------------
  RCLCPP_INFO(logger, "=== FINE phase: target=%s ===", fine_goal_frame.c_str());

  // 微動は最初から遅く（安全寄り）
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

  if (!plan_and_execute_pose(logger, move_group, fine_goal_pose, ee_link)) {
    RCLCPP_ERROR(logger, "FINE execute failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "DONE: coarse-to-fine go_to_leaf finished.");
  rclcpp::shutdown();
  return 0;
}
