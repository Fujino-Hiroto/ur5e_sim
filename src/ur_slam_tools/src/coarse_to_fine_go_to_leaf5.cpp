// (patch) add COARSE retry + FINE orientation mode + eef_step fallback
// (patch) add FINE orientation blend mode (fixed <-> lookat)
// NOTE: FINE is changed to Cartesian straight approach using computeCartesianPath.
// coarse_to_fine_go_to_leaf5.cpp
//
// Minimal Coarse->Fine->View for noise impact experiments.
// Policy: Option B -> COARSE goal is a PoseTarget (position + orientation).
//
// - No segmented motion
// - No runtime obstacle monitor / stop()
// - No path constraints
// - No RTAB-Map param switching
//
// Keep node split (app_node + moveit_node) to avoid executor conflicts.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/parameter_client.hpp>
#include <set>
#include <vector>
#include <future>
#include <thread>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

static geometry_msgs::msg::Vector3 v3(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

static geometry_msgs::msg::Vector3 v3_sub(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static geometry_msgs::msg::Vector3 v3_scale(const geometry_msgs::msg::Vector3 &a, double s)
{
  return v3(a.x * s, a.y * s, a.z * s);
}

static double v3_norm(const geometry_msgs::msg::Vector3 &a)
{
  return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

static geometry_msgs::msg::Vector3 v3_unit(const geometry_msgs::msg::Vector3 &a)
{
  const double n = v3_norm(a);
  if (n < 1e-12) return v3(0, 0, 0);
  return v3_scale(a, 1.0 / n);
}

static geometry_msgs::msg::Vector3 v3_cross(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b)
{
  return v3(a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}

static bool lookup_tf_blocking(
    tf2_ros::Buffer &tf_buffer,
    const rclcpp::Node::SharedPtr &node,
    const std::string &target,
    const std::string &source,
    geometry_msgs::msg::TransformStamped &out,
    double timeout_sec)
{
  const auto logger = node->get_logger();
  const auto clock = node->get_clock();
  const auto start = clock->now();
  rclcpp::Rate rate(50);

  while (rclcpp::ok())
  {
    try
    {
      out = tf_buffer.lookupTransform(target, source, tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
                           "Waiting TF %s -> %s: %s",
                           target.c_str(), source.c_str(), ex.what());
    }

    if ((clock->now() - start).seconds() > timeout_sec)
      return false;

    rate.sleep();
  }
  return false;
}

static geometry_msgs::msg::Vector3 pos_from_tf(const geometry_msgs::msg::TransformStamped &T)
{
  return v3(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
}

// Look-at pose builder: +Z points to target
static geometry_msgs::msg::Quaternion quat_from_R(
    const geometry_msgs::msg::Vector3 &x_axis,
    const geometry_msgs::msg::Vector3 &y_axis,
    const geometry_msgs::msg::Vector3 &z_axis)
{
  const double r00 = x_axis.x, r01 = y_axis.x, r02 = z_axis.x;
  const double r10 = x_axis.y, r11 = y_axis.y, r12 = z_axis.y;
  const double r20 = x_axis.z, r21 = y_axis.z, r22 = z_axis.z;

  geometry_msgs::msg::Quaternion q;
  const double tr = r00 + r11 + r22;

  if (tr > 0.0)
  {
    const double S = std::sqrt(tr + 1.0) * 2.0;
    q.w = 0.25 * S;
    q.x = (r21 - r12) / S;
    q.y = (r02 - r20) / S;
    q.z = (r10 - r01) / S;
  }
  else if ((r00 > r11) && (r00 > r22))
  {
    const double S = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    q.w = (r21 - r12) / S;
    q.x = 0.25 * S;
    q.y = (r01 + r10) / S;
    q.z = (r02 + r20) / S;
  }
  else if (r11 > r22)
  {
    const double S = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    q.w = (r02 - r20) / S;
    q.x = (r01 + r10) / S;
    q.y = 0.25 * S;
    q.z = (r12 + r21) / S;
  }
  else
  {
    const double S = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    q.w = (r10 - r01) / S;
    q.x = (r02 + r20) / S;
    q.y = (r12 + r21) / S;
    q.z = 0.25 * S;
  }
  return q;
}

static geometry_msgs::msg::PoseStamped make_lookat_pose(
    const std::string &frame_id,
    const geometry_msgs::msg::Vector3 &target_pos_world,
    const geometry_msgs::msg::Vector3 &world_up,
    const geometry_msgs::msg::Vector3 &desired_pos_world)
{
  geometry_msgs::msg::Vector3 z = v3_sub(target_pos_world, desired_pos_world);
  z = v3_unit(z);
  if (v3_norm(z) < 1e-9) z = v3(0, 0, 1);

  geometry_msgs::msg::Vector3 x = v3_cross(world_up, z);
  if (v3_norm(x) < 1e-9) x = v3_cross(v3(0, 1, 0), z);
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

// Convert goal pose expressed for camera_frame into ee_link pose (world_frame basis).
// If camera_frame == ee_link, returns unchanged.
static bool convert_goal_pose_for_ee(
    tf2_ros::Buffer &tf_buffer,
    const rclcpp::Logger &logger,
    const std::string &camera_frame,
    const std::string &ee_link,
    const geometry_msgs::msg::PoseStamped &goal_in_camera,
    geometry_msgs::msg::PoseStamped &out_goal)
{
  if (camera_frame.empty() || camera_frame == ee_link)
  {
    out_goal = goal_in_camera;
    return true;
  }

  try
  {
    const auto cam_T_ee_msg = tf_buffer.lookupTransform(
        camera_frame, ee_link, tf2::TimePointZero);
    tf2::Transform cam_T_ee;
    tf2::fromMsg(cam_T_ee_msg.transform, cam_T_ee);

    tf2::Transform world_T_cam;
    tf2::fromMsg(goal_in_camera.pose, world_T_cam);
    const tf2::Transform world_T_ee = world_T_cam * cam_T_ee;

    out_goal = goal_in_camera;
    out_goal.pose.position.x = world_T_ee.getOrigin().x();
    out_goal.pose.position.y = world_T_ee.getOrigin().y();
    out_goal.pose.position.z = world_T_ee.getOrigin().z();
    out_goal.pose.orientation = tf2::toMsg(world_T_ee.getRotation());
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(logger, "Failed to convert goal pose from %s to %s: %s",
                camera_frame.c_str(), ee_link.c_str(), e.what());
    out_goal = goal_in_camera;
    return false;
  }
}

// ---- Diagnostics helpers ----
static std::vector<double> get_current_joint_positions(
    const moveit::core::RobotStatePtr& state,
    const std::vector<std::string>& joint_names)
{
  std::vector<double> q;
  q.resize(joint_names.size(), 0.0);
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    q[i] = state->getVariablePosition(joint_names[i]);
  }
  return q;
}

static double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b)
{
  const size_t n = std::min(a.size(), b.size());
  double m = 0.0;
  for (size_t i = 0; i < n; ++i)
  {
    m = std::max(m, std::abs(a[i] - b[i]));
  }
  return m;
}

static void log_joint_diff_if_large(
    const rclcpp::Logger& logger,
    const std::vector<std::string>& names,
    const std::vector<double>& a,
    const std::vector<double>& b,
    double warn_thr_rad,
    const char* label_a,
    const char* label_b)
{
  const double m = max_abs_diff(a, b);
  if (m < warn_thr_rad) return;

  RCLCPP_WARN(logger,
              "Start-state mismatch detected (max |%s-%s| = %.6f rad, thr=%.6f rad). "
              "This can cause an initial 'jump' before the Cartesian segment.",
              label_a, label_b, m, warn_thr_rad);
  for (size_t i = 0; i < names.size() && i < a.size() && i < b.size(); ++i)
  {
    const double d = a[i] - b[i];
    if (std::abs(d) >= warn_thr_rad)
    {
      RCLCPP_WARN(logger, "  joint=%s  %s=%.6f  %s=%.6f  diff=%.6f",
                  names[i].c_str(), label_a, a[i], label_b, b[i], d);
    }
  }
}

static void copy_moveit_description_params(
    const rclcpp::Node::SharedPtr& app_node,
    const rclcpp::Node::SharedPtr& moveit_node)
{
  (void)app_node;
  const auto logger = moveit_node->get_logger();

  auto params_client = std::make_shared<rclcpp::AsyncParametersClient>(moveit_node, "move_group");

  if (!params_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(logger, "Parameter service for /move_group not available; MoveIt params not copied.");
    return;
  }

  std::set<std::string> names;

  auto add_prefix = [&](const std::string &prefix)
  {
    auto list_fut = params_client->list_parameters({prefix}, 50);
    if (list_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_WARN(logger, "list_parameters timeout for prefix: %s", prefix.c_str());
      return;
    }
    const auto list = list_fut.get();
    names.insert(list.names.begin(), list.names.end());
  };

  add_prefix("robot_description");
  add_prefix("robot_description_semantic");
  add_prefix("robot_description_kinematics");
  add_prefix("robot_description_planning");

  if (names.empty())
  {
    RCLCPP_WARN(logger, "list_parameters returned empty; fallback to direct get.");

    std::vector<std::string> fallback = {
      "robot_description",
      "robot_description_semantic",
      "robot_description_kinematics",
      "robot_description_planning"
    };

    auto get_fut = params_client->get_parameters(fallback);
    if (get_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_WARN(logger, "get_parameters timeout for fallback keys.");
      return;
    }

    const auto params = get_fut.get();
    (void)moveit_node->set_parameters(params);
    RCLCPP_INFO(logger, "Copied %zu MoveIt description parameters from /move_group (fallback).", params.size());
    return;
  }

  std::vector<std::string> name_vec(names.begin(), names.end());
  auto get_fut = params_client->get_parameters(name_vec);
  if (get_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(logger, "get_parameters timeout for MoveIt description parameters.");
    return;
  }

  const auto params = get_fut.get();
  (void)moveit_node->set_parameters(params);
  RCLCPP_INFO(logger, "Copied %zu MoveIt description parameters from /move_group.", params.size());
}

static geometry_msgs::msg::Vector3 compute_leaf_pre_position(
    const geometry_msgs::msg::Vector3 &cam_pos_world,
    const geometry_msgs::msg::Vector3 &leaf_pos_world,
    double pre_back_m,
    double pre_down_m,
    double min_xy_norm_eps = 1e-6)
{
  geometry_msgs::msg::Vector3 v = v3_sub(leaf_pos_world, cam_pos_world);

  geometry_msgs::msg::Vector3 vxy = v3(v.x, v.y, 0.0);
  const double nxy = v3_norm(vxy);

  geometry_msgs::msg::Vector3 dir_xy = v3(1, 0, 0);
  if (nxy > min_xy_norm_eps)
    dir_xy = v3_scale(vxy, 1.0 / nxy);

  geometry_msgs::msg::Vector3 pre = leaf_pos_world;
  pre = v3_sub(pre, v3_scale(dir_xy, pre_back_m));
  pre = v3_sub(pre, v3(0, 0, pre_down_m));
  return pre;
}

static geometry_msgs::msg::Quaternion slerp_quat_msg(
    const geometry_msgs::msg::Quaternion& qa_msg,
    const geometry_msgs::msg::Quaternion& qb_msg,
    double t)
{
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;

  tf2::Quaternion qa, qb;
  tf2::fromMsg(qa_msg, qa);
  tf2::fromMsg(qb_msg, qb);

  qa.normalize();
  qb.normalize();

  tf2::Quaternion q = qa.slerp(qb, t);
  q.normalize();

  return tf2::toMsg(q);
}

static bool plan_and_execute_pose_with_retry(
    const rclcpp::Logger &logger,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::PoseStamped &goal_ee,
    const std::string &ee_link_for_target,
    int max_tries,
    double sleep_sec)
{
  if (max_tries < 1) max_tries = 1;
  if (sleep_sec < 0.0) sleep_sec = 0.0;

  for (int i = 1; i <= max_tries; ++i)
  {
    if (i > 1)
    {
      RCLCPP_WARN(logger, "COARSE retry %d/%d ...", i, max_tries);
      if (sleep_sec > 0.0)
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_sec));
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal_ee, ee_link_for_target);

    auto ret = move_group.plan(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(logger, "COARSE PLAN failed (MoveItErrorCode=%d).", ret.val);
      continue;
    }

    ret = move_group.execute(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(logger, "COARSE EXEC failed (MoveItErrorCode=%d).", ret.val);
      continue;
    }

    return true;
  }
  return false;
}

// Cartesian straight-line motion (EEF) from current pose to goal pose.
static bool execute_cartesian_to_pose(
    const rclcpp::Logger &logger,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::PoseStamped &goal_ee,
    const std::string &ee_link,
    double eef_step,
    double jump_threshold,
    double min_fraction,
    bool avoid_collisions,
    double vel_scale,
    double acc_scale,
    double p0_warn_rad)
{
  move_group.setStartStateToCurrentState();
  move_group.clearPoseTargets();

  const auto start_ps = move_group.getCurrentPose(ee_link);

  RCLCPP_INFO(logger, "EEF start frame=%s goal frame=%s",
              start_ps.header.frame_id.c_str(),
              goal_ee.header.frame_id.c_str());

  const auto ref = move_group.getPoseReferenceFrame();
  if (start_ps.header.frame_id != ref || goal_ee.header.frame_id != ref)
  {
    RCLCPP_ERROR(logger, "Frame mismatch: ref=%s start=%s goal=%s",
                 ref.c_str(),
                 start_ps.header.frame_id.c_str(),
                 goal_ee.header.frame_id.c_str());
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(1);
  waypoints.push_back(goal_ee.pose);

  moveit_msgs::msg::RobotTrajectory traj_msg;
  const double fraction =
      move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, traj_msg, avoid_collisions);

  auto &jt = traj_msg.joint_trajectory;

  if (jt.joint_names.empty())
  {
    auto robot_model_for_names = move_group.getRobotModel();
    auto jmg = robot_model_for_names
                 ? robot_model_for_names->getJointModelGroup(move_group.getName())
                 : nullptr;

    if (!jmg)
    {
      RCLCPP_ERROR(logger, "JointModelGroup is null; cannot fill joint_names.");
      return false;
    }

    jt.joint_names = jmg->getVariableNames();
    RCLCPP_WARN(logger, "traj joint_names was empty -> filled from JointModelGroup (%zu joints).",
                jt.joint_names.size());
  }

  if (jt.points.empty())
  {
    RCLCPP_ERROR(logger, "Cartesian trajectory has 0 points.");
    return false;
  }

  const std::vector<double> traj_first_pos = jt.points.front().positions;

  RCLCPP_INFO(logger, "CartesianPath fraction=%.3f (min=%.3f) eef_step=%.4f jump=%.3f",
              fraction, min_fraction, eef_step, jump_threshold);

  const double last_t_before =
      jt.points.empty() ? -1.0 : rclcpp::Duration(jt.points.back().time_from_start).seconds();
  RCLCPP_INFO(logger, "traj(before retime): joints=%zu points=%zu last_t=%.6f",
              jt.joint_names.size(), jt.points.size(), last_t_before);

  if (fraction < min_fraction)
  {
    RCLCPP_ERROR(logger, "Cartesian path fraction too low: %.3f < %.3f", fraction, min_fraction);
    return false;
  }

  const size_t nj = jt.joint_names.size();
  const size_t np = jt.points.front().positions.size();
  if (nj == 0 || np == 0 || nj != np)
  {
    RCLCPP_ERROR(logger, "Trajectory dimension mismatch: joint_names=%zu point.positions=%zu", nj, np);
    return false;
  }

  auto robot_model = move_group.getRobotModel();
  if (!robot_model)
  {
    RCLCPP_ERROR(logger, "RobotModel is null; abort.");
    return false;
  }

  auto current_state = move_group.getCurrentState(5.0);
  if (!current_state)
  {
    RCLCPP_ERROR(logger, "CurrentState is null; abort.");
    return false;
  }

  {
    const auto q_now = get_current_joint_positions(current_state, jt.joint_names);
    if (traj_first_pos.size() == q_now.size())
    {
      log_joint_diff_if_large(logger, jt.joint_names, q_now, traj_first_pos,
                              p0_warn_rad, "q_now", "traj_p0");
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.positions.resize(jt.joint_names.size());
  for (size_t i = 0; i < jt.joint_names.size(); ++i)
  {
    const auto *jm = current_state->getJointModel(jt.joint_names[i]);
    if (!jm)
    {
      RCLCPP_ERROR(logger, "CurrentState has no joint model for '%s'", jt.joint_names[i].c_str());
      return false;
    }
    p0.positions[i] = current_state->getVariablePosition(jt.joint_names[i]);
  }
  p0.velocities.assign(jt.joint_names.size(), 0.0);
  p0.accelerations.assign(jt.joint_names.size(), 0.0);
  p0.time_from_start = rclcpp::Duration(0, 0);

  jt.points.insert(jt.points.begin(), p0);

  for (auto &pt : jt.points)
  {
    if (pt.velocities.size() != nj)     pt.velocities.assign(nj, 0.0);
    if (pt.accelerations.size() != nj) pt.accelerations.assign(nj, 0.0);
  }

  robot_trajectory::RobotTrajectory rt(robot_model, move_group.getName());
  rt.setRobotTrajectoryMsg(*current_state, traj_msg);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, vel_scale, acc_scale))
  {
    RCLCPP_ERROR(logger, "Time parameterization failed; abort.");
    return false;
  }
  rt.getRobotTrajectoryMsg(traj_msg);

  auto &jt_after = traj_msg.joint_trajectory;
  const double last_t_after =
      jt_after.points.empty() ? -1.0 : rclcpp::Duration(jt_after.points.back().time_from_start).seconds();
  RCLCPP_INFO(logger, "traj(after retime): joints=%zu points=%zu last_t=%.6f",
              jt_after.joint_names.size(), jt_after.points.size(), last_t_after);
  if (last_t_after <= 1e-3)
  {
    RCLCPP_ERROR(logger, "Trajectory duration too small after retime (%.6f).", last_t_after);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj_msg;

  auto ret = move_group.execute(plan);
  if (ret != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Cartesian EXEC failed (MoveItErrorCode=%d).", ret.val);
    return false;
  }
  return true;
}

static bool execute_cartesian_waypoints(
    const rclcpp::Logger &logger,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::string &ee_link,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    double eef_step,
    double jump_threshold,
    double min_fraction,
    bool avoid_collisions,
    double vel_scale,
    double acc_scale,
    double p0_warn_rad)
{
  (void)ee_link;
  move_group.setStartStateToCurrentState();
  move_group.clearPoseTargets();

  moveit_msgs::msg::RobotTrajectory traj_msg;
  const double fraction =
      move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, traj_msg, avoid_collisions);

  auto &jt = traj_msg.joint_trajectory;
  if (jt.joint_names.empty())
  {
    auto robot_model_for_names = move_group.getRobotModel();
    auto jmg = robot_model_for_names
                 ? robot_model_for_names->getJointModelGroup(move_group.getName())
                 : nullptr;

    if (!jmg)
    {
      RCLCPP_ERROR(logger, "JointModelGroup is null; cannot fill joint_names.");
      return false;
    }

    jt.joint_names = jmg->getVariableNames();
    RCLCPP_WARN(logger, "traj joint_names was empty -> filled from JointModelGroup (%zu joints).",
                jt.joint_names.size());
  }
  if (jt.points.empty())
  {
    RCLCPP_ERROR(logger, "Cartesian trajectory has 0 points.");
    return false;
  }

  const std::vector<double> traj_first_pos = jt.points.front().positions;

  const size_t nj = jt.joint_names.size();
  const size_t np = jt.points.front().positions.size();
  if (nj == 0 || np == 0 || nj != np)
  {
    RCLCPP_ERROR(logger, "Trajectory dimension mismatch: joint_names=%zu point.positions=%zu", nj, np);
    return false;
  }
  if (fraction < min_fraction)
  {
    RCLCPP_ERROR(logger, "Cartesian path fraction too low: %.3f < %.3f", fraction, min_fraction);
    return false;
  }

  auto current_state = move_group.getCurrentState(5.0);
  if (!current_state) return false;

  {
    const auto q_now = get_current_joint_positions(current_state, jt.joint_names);
    if (traj_first_pos.size() == q_now.size())
      log_joint_diff_if_large(logger, jt.joint_names, q_now, traj_first_pos,
                              p0_warn_rad, "q_now", "traj_p0");
  }

  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.positions.resize(jt.joint_names.size());
  for (size_t i = 0; i < jt.joint_names.size(); ++i)
    p0.positions[i] = current_state->getVariablePosition(jt.joint_names[i]);
  p0.velocities.assign(jt.joint_names.size(), 0.0);
  p0.accelerations.assign(jt.joint_names.size(), 0.0);
  p0.time_from_start = rclcpp::Duration(0, 0);
  jt.points.insert(jt.points.begin(), p0);

  for (auto &pt : jt.points)
  {
    if (pt.velocities.size() != nj)     pt.velocities.assign(nj, 0.0);
    if (pt.accelerations.size() != nj) pt.accelerations.assign(nj, 0.0);
  }

  auto robot_model = move_group.getRobotModel();
  if (!robot_model) return false;
  robot_trajectory::RobotTrajectory rt(robot_model, move_group.getName());
  rt.setRobotTrajectoryMsg(*current_state, traj_msg);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, vel_scale, acc_scale)) return false;
  rt.getRobotTrajectoryMsg(traj_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj_msg;
  auto ret = move_group.execute(plan);
  if (ret != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Cartesian EXEC failed (MoveItErrorCode=%d).", ret.val);
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto app_node = rclcpp::Node::make_shared("coarse_to_fine_go_to_leaf5");
  const auto logger = app_node->get_logger();

  auto moveit_node = rclcpp::Node::make_shared(
      "coarse_to_fine_go_to_leaf5_moveit",
      rclcpp::NodeOptions().allow_undeclared_parameters(true)
                          .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(app_node);
  exec.add_node(moveit_node);
  std::thread spin_thread([&]() { exec.spin(); });

  auto cleanup_and_exit = [&](int code) -> int
  {
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return code;
  };

  if (!app_node->has_parameter("use_sim_time"))
    app_node->declare_parameter<bool>("use_sim_time", true);

  bool use_sim_time = true;
  app_node->get_parameter("use_sim_time", use_sim_time);
  if (!moveit_node->has_parameter("use_sim_time"))
    moveit_node->declare_parameter<bool>("use_sim_time", use_sim_time);
  moveit_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

  copy_moveit_description_params(app_node, moveit_node);

  std::string kin_solver;
  if (moveit_node->get_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver", kin_solver))
  {
    RCLCPP_INFO(logger, "kinematics_solver=%s", kin_solver.c_str());
  }
  else
  {
    RCLCPP_ERROR(logger, "kinematics parameters not set on moveit_node. Abort.");
    return cleanup_and_exit(1);
  }

  const std::string world_frame      = app_node->declare_parameter<std::string>("world_frame", "world");
  const std::string camera_frame     = app_node->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  const std::string ee_link          = app_node->declare_parameter<std::string>("ee_link", "tool0");

  const std::string leaf_center_frame = app_node->declare_parameter<std::string>("leaf_center_frame", "leaf_target");
  const std::string fine_goal_frame   = app_node->declare_parameter<std::string>("fine_goal_frame", "leaf_target_shot");
  const std::string view_goal_frame   = app_node->declare_parameter<std::string>("view_goal_frame", "leaf_target_view");

  const bool exit_after_coarse = app_node->declare_parameter<bool>("exit_after_coarse", false);

  const double tf_timeout_sec   = app_node->declare_parameter<double>("tf_timeout_sec", 10.0);

  const double pre_back_m = app_node->declare_parameter<double>("pre_back_m", 0.30);
  const double pre_down_m = app_node->declare_parameter<double>("pre_down_m", 0.15);

  const double coarse_planning_time = app_node->declare_parameter<double>("coarse_planning_time", 15.0);
  const double fine_planning_time   = app_node->declare_parameter<double>("fine_planning_time", 10.0);
  const double view_planning_time   = app_node->declare_parameter<double>("view_planning_time", 5.0);

  const int coarse_max_tries = app_node->declare_parameter<int>("coarse_max_tries", 5);
  const double coarse_retry_sleep_sec =
      app_node->declare_parameter<double>("coarse_retry_sleep_sec", 0.2);

  const std::string fine_orient_mode =
      app_node->declare_parameter<std::string>("fine_orient_mode", "lookat");
  const double fine_orient_blend_alpha =
      app_node->declare_parameter<double>("fine_orient_blend_alpha", 0.3);

  const double coarse_vel = app_node->declare_parameter<double>("coarse_vel_scale", 0.05);
  const double coarse_acc = app_node->declare_parameter<double>("coarse_acc_scale", 0.05);
  const double fine_vel   = app_node->declare_parameter<double>("fine_vel_scale", 0.03);
  const double fine_acc   = app_node->declare_parameter<double>("fine_acc_scale", 0.03);
  const double view_vel   = app_node->declare_parameter<double>("view_vel_scale", 0.01);
  const double view_acc   = app_node->declare_parameter<double>("view_acc_scale", 0.01);

  const double fine_cart_eef_step     = app_node->declare_parameter<double>("fine_cartesian_eef_step", 0.005);
  const double fine_cart_jump_thresh  = app_node->declare_parameter<double>("fine_cartesian_jump_threshold", 0.0);
  const double fine_cart_min_fraction = app_node->declare_parameter<double>("fine_cartesian_min_fraction", 0.95);
  const bool   fine_cart_avoid_coll   = app_node->declare_parameter<bool>("fine_cartesian_avoid_collisions", true);

  const double view_cart_eef_step     = app_node->declare_parameter<double>("view_cartesian_eef_step", 0.001);
  const double view_cart_min_fraction = app_node->declare_parameter<double>("view_cartesian_min_fraction", 0.90);
  const bool   view_cart_avoid_coll   = app_node->declare_parameter<bool>("view_cartesian_avoid_collisions", false);

  const int    state_sync_sleep_ms = app_node->declare_parameter<int>("state_sync_sleep_ms", 200);
  const double p0_warn_rad         = app_node->declare_parameter<double>("p0_warn_rad", 0.02);
  const std::vector<double> fine_cart_eef_step_candidates =
      app_node->declare_parameter<std::vector<double>>(
          "fine_cartesian_eef_step_candidates",
          std::vector<double>{});

  RCLCPP_INFO(logger, "use_sim_time=%s world_frame=%s ee_link=%s camera_frame=%s",
              use_sim_time ? "true" : "false",
              world_frame.c_str(), ee_link.c_str(), camera_frame.c_str());

  tf2_ros::Buffer tf_buffer(app_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, app_node, /*spin_thread=*/false);

  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(moveit_node, PLANNING_GROUP);
  move_group.setPoseReferenceFrame(world_frame);
  move_group.setEndEffectorLink(ee_link);

  RCLCPP_INFO(logger, "Planning group: %s", PLANNING_GROUP.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  const geometry_msgs::msg::Vector3 world_up = v3(0, 0, 1);

  RCLCPP_INFO(logger, "=== COARSE (PoseTarget): pre approach ===");
  move_group.setPlanningTime(coarse_planning_time);
  move_group.setMaxVelocityScalingFactor(coarse_vel);
  move_group.setMaxAccelerationScalingFactor(coarse_acc);

  geometry_msgs::msg::TransformStamped Tw_leaf;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, leaf_center_frame, Tw_leaf, tf_timeout_sec))
  {
    RCLCPP_ERROR(logger, "TF timeout: %s <- %s", world_frame.c_str(), leaf_center_frame.c_str());
    return cleanup_and_exit(1);
  }
  const auto leaf_pos_world = pos_from_tf(Tw_leaf);

  const auto cur_cam_ps = move_group.getCurrentPose(camera_frame.empty() ? ee_link : camera_frame);
  const auto cam_pos_world = v3(cur_cam_ps.pose.position.x, cur_cam_ps.pose.position.y, cur_cam_ps.pose.position.z);

  const auto pre_pos_world = compute_leaf_pre_position(cam_pos_world, leaf_pos_world, pre_back_m, pre_down_m);

  const auto coarse_goal_cam = make_lookat_pose(world_frame, leaf_pos_world, world_up, pre_pos_world);

  geometry_msgs::msg::PoseStamped coarse_goal_ee;
  if (!convert_goal_pose_for_ee(tf_buffer, logger, camera_frame, ee_link, coarse_goal_cam, coarse_goal_ee))
  {
    RCLCPP_ERROR(logger, "COARSE: convert_goal_pose_for_ee failed (camera_frame=%s ee_link=%s).",
                camera_frame.c_str(), ee_link.c_str());
    return cleanup_and_exit(1);
  }

  if (!plan_and_execute_pose_with_retry(logger, move_group, coarse_goal_ee, ee_link,
                                        coarse_max_tries, coarse_retry_sleep_sec))
  {
    RCLCPP_ERROR(logger, "COARSE failed.");
    return cleanup_and_exit(1);
  }

  RCLCPP_INFO(logger, "COARSE success.");

  if (state_sync_sleep_ms > 0)
    rclcpp::sleep_for(std::chrono::milliseconds(state_sync_sleep_ms));
  move_group.setStartStateToCurrentState();

  if (exit_after_coarse)
  {
    RCLCPP_INFO(logger, "exit_after_coarse=true -> stop after COARSE.");
    return cleanup_and_exit(0);
  }

  RCLCPP_INFO(logger, "=== FINE (Cartesian): go to %s ===", fine_goal_frame.c_str());
  move_group.setPlanningTime(fine_planning_time);
  move_group.setMaxVelocityScalingFactor(fine_vel);
  move_group.setMaxAccelerationScalingFactor(fine_acc);

  geometry_msgs::msg::TransformStamped Tw_shot;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, fine_goal_frame, Tw_shot, tf_timeout_sec))
  {
    RCLCPP_ERROR(logger, "TF timeout: %s <- %s", world_frame.c_str(), fine_goal_frame.c_str());
    return cleanup_and_exit(1);
  }
  const auto shot_pos_world = pos_from_tf(Tw_shot);

  const auto fine_goal_cam = make_lookat_pose(world_frame, leaf_pos_world, world_up, shot_pos_world);

  geometry_msgs::msg::PoseStamped fine_goal_ee;
  if (!convert_goal_pose_for_ee(tf_buffer, logger, camera_frame, ee_link, fine_goal_cam, fine_goal_ee))
  {
    RCLCPP_ERROR(logger, "FINE: convert_goal_pose_for_ee failed (camera_frame=%s ee_link=%s).",
                camera_frame.c_str(), ee_link.c_str());
    return cleanup_and_exit(1);
  }

  const auto cur_ee_ps = move_group.getCurrentPose(ee_link);
  const auto q_cur = cur_ee_ps.pose.orientation;
  const auto q_lookat = fine_goal_ee.pose.orientation;

  if (fine_orient_mode == "fixed")
  {
    fine_goal_ee.pose.orientation = q_cur;
    RCLCPP_INFO(logger, "FINE orient_mode=fixed: orientation copied from current EE.");
  }
  else if (fine_orient_mode == "blend")
  {
    double a = fine_orient_blend_alpha;
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;
    fine_goal_ee.pose.orientation = slerp_quat_msg(q_cur, q_lookat, a);
    RCLCPP_INFO(logger, "FINE orient_mode=blend: slerp(current->lookat, alpha=%.3f).", a);
  }

  std::vector<double> steps;
  if (!fine_cart_eef_step_candidates.empty())
    steps = fine_cart_eef_step_candidates;
  else
    steps = {fine_cart_eef_step};

  bool fine_ok = false;
  for (size_t i = 0; i < steps.size(); ++i)
  {
    const double step = steps[i];
    if (step <= 0.0)
    {
      RCLCPP_WARN(logger, "Skip invalid eef_step=%.6f", step);
      continue;
    }

    if (steps.size() > 1)
    {
      RCLCPP_INFO(logger, "FINE attempt with eef_step=%.6f (%zu/%zu)",
                  step, i + 1, steps.size());
    }

    if (execute_cartesian_to_pose(logger, move_group, fine_goal_ee, ee_link,
                                  step,
                                  fine_cart_jump_thresh,
                                  fine_cart_min_fraction,
                                  fine_cart_avoid_coll,
                                  fine_vel,
                                  fine_acc,
                                  p0_warn_rad))
    {
      fine_ok = true;
      break;
    }
  }

  if (!fine_ok)
  {
    RCLCPP_ERROR(logger, "FINE failed (all eef_step attempts exhausted).\n");
    return cleanup_and_exit(1);
  }

  RCLCPP_INFO(logger, "FINE success.");

  RCLCPP_INFO(logger, "=== VIEW: orientation-only align to %s ===", view_goal_frame.c_str());
  move_group.setPlanningTime(view_planning_time);
  move_group.setMaxVelocityScalingFactor(view_vel);
  move_group.setMaxAccelerationScalingFactor(view_acc);

  geometry_msgs::msg::TransformStamped Tw_view;
  if (!lookup_tf_blocking(tf_buffer, app_node, world_frame, view_goal_frame, Tw_view, tf_timeout_sec))
  {
    RCLCPP_WARN(logger, "TF timeout: %s <- %s. Skip VIEW.",
                world_frame.c_str(), view_goal_frame.c_str());
  }
  else
  {
    const auto cur_cam_ps2 = move_group.getCurrentPose(camera_frame.empty() ? ee_link : camera_frame);

    geometry_msgs::msg::PoseStamped view_goal_cam;
    view_goal_cam.header.frame_id = world_frame;
    view_goal_cam.pose.position = cur_cam_ps2.pose.position;
    view_goal_cam.pose.orientation = Tw_view.transform.rotation;

    geometry_msgs::msg::PoseStamped view_goal_ee;
    if (!convert_goal_pose_for_ee(tf_buffer, logger, camera_frame, ee_link, view_goal_cam, view_goal_ee))
    {
      RCLCPP_WARN(logger, "VIEW: convert_goal_pose_for_ee failed (camera_frame=%s ee_link=%s). Skip VIEW.",
                  camera_frame.c_str(), ee_link.c_str());
    }
    else
    {
      geometry_msgs::msg::Pose p1 = view_goal_ee.pose;
      geometry_msgs::msg::Pose p2 = view_goal_ee.pose;
      p1.position.z += 1e-4;
      p2.position.z -= 1e-4;

      std::vector<geometry_msgs::msg::Pose> wps;
      wps.push_back(move_group.getCurrentPose(ee_link).pose);
      wps.push_back(p1);
      wps.push_back(p2);

      if (!execute_cartesian_waypoints(logger, move_group, ee_link,
                                       wps,
                                       view_cart_eef_step,
                                       0.0,
                                       view_cart_min_fraction,
                                       view_cart_avoid_coll,
                                       view_vel, view_acc,
                                       p0_warn_rad))
      {
        RCLCPP_WARN(logger, "VIEW failed (best-effort).\n");
      }
      else
      {
        RCLCPP_INFO(logger, "VIEW success.");
      }
    }
  }

  RCLCPP_INFO(logger, "DONE: leaf5 coarse->fine->view finished.");
  return cleanup_and_exit(0);
}
