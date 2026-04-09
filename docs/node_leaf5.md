# coarse_to_fine_go_to_leaf5 仕様

## 動作フロー

```
[COARSE]  中継地点へ OMPL で移動（ompl_direct 以外）
    ↓
[FINE]    leaf_target_shot へ移動（planning_mode で方式が変わる）
    ↓
[VIEW]    leaf_target_view の姿勢に向きを合わせる（Cartesian）
```

## planning_mode

| モード | COARSE | FINE | 特徴 |
|---|---|---|---|
| `hybrid` | OMPL | Cartesian（computeCartesianPath） | IK ブランチ跳び起きにくい |
| `ompl_two_stage` | OMPL | OMPL | 卒論で使用・時間がかかった |
| `ompl_direct` | スキップ | OMPL | COARSE なし・初期姿勢から直接 |

## fine_orient_mode

| モード | FINE ゴール姿勢の決め方 |
|---|---|
| `lookat` | shot 地点から葉を見る方向（make_lookat_pose） |
| `fixed` | COARSE 到達時の姿勢をそのまま引き継ぐ |
| `blend` | COARSE 到達姿勢と lookat の slerp（alpha で制御） |
| `seed_joints` | COARSE 到達関節状態を IK シードとして FINE に渡す（未実装） |

## 主要パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `planning_mode` | `hybrid` | 上記参照 |
| `fine_orient_mode` | `lookat` | 上記参照 |
| `fine_orient_blend_alpha` | `0.3` | blend 時の lookat 混合率（0=fixed, 1=lookat） |
| `pre_back_m` | `0.30` | COARSE 中継地点：葉から後退する距離 |
| `pre_down_m` | `0.15` | COARSE 中継地点：葉から下げる距離 |
| `coarse_vel_scale` | `0.05` | COARSE 速度スケール |
| `fine_vel_scale` | `0.03` | FINE 速度スケール |
| `csv_enable` | `true` | CSV 記録の有効化 |
| `csv_path` | `results/leaf5_log.csv` | CSV 出力先 |
| `csv_run_id` | `""` | ランID（空なら UTC 時刻） |

## CSV 出力フォーマット

カラム: `iso8601_utc, unix_ms, ros_time_sec, run_id, phase, event, status, eef_step, fraction, points, last_t_sec, detail`

`detail` カラムの形式（スペース区切り key=value）:
```
plan_t=X exec_t=X joint_travel=X max_joint_step=X pos_err_m=X ang_err_deg=X
```

## IK ソルバ設定

`src/ur_slam_bringup/config/ur5e/moveit_kinematics.yaml`
```yaml
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
kinematics_solver_search_resolution: 0.005
kinematics_solver_timeout: 0.005
kinematics_solver_attempts: 3
```

## 実行コマンド例（hybrid モード・mid ノイズ）

```bash
ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \
  -p planning_mode:=hybrid \
  -p use_sim_time:=true \
  -p world_frame:=base_link \
  -p ee_link:=tool0 \
  -p camera_frame:=camera_color_optical_frame \
  -p leaf_center_frame:=leaf_target \
  -p fine_goal_frame:=leaf_target_shot \
  -p view_goal_frame:=leaf_target_view \
  -p tf_timeout_sec:=10.0 \
  -p coarse_vel_scale:=0.05 -p coarse_acc_scale:=0.05 \
  -p fine_vel_scale:=0.03  -p fine_acc_scale:=0.03 \
  -p fine_orient_mode:=blend -p fine_orient_blend_alpha:=0.7 \
  -p csv_enable:=true -p csv_path:=results/leaf5_log.csv \
  -p csv_run_id:=test_run01
```
