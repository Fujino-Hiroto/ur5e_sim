# run_bench.sh 仕様書

## 概要

1コマンドで指定条件の実験をN回連続自動実行するスクリプト。
毎試行ごとに全プロセスを起動→実行→終了することで試行間の公平性を確保する。

## 使い方

```bash
./scripts/run_bench.sh <CONDITION> <NOISE> [N_TRIALS]
```

### 引数

| 引数 | 値 | 説明 |
|---|---|---|
| CONDITION | A / B / C | 経路計画方式（後述） |
| NOISE | clean / weak / medium / strong | ノイズ条件（後述） |
| N_TRIALS | 整数（デフォルト: 10） | 試行回数 |

### 実行例

```bash
./scripts/run_bench.sh A weak 10
./scripts/run_bench.sh B strong 5
./scripts/run_bench.sh A clean    # N_TRIALSを省略すると10回
```

---

## 起動プロセスと順序

各試行で以下の順番で起動し、実験完了後に全プロセスを終了する。

| # | プロセス | 起動方法 | 完了判定トピック |
|---|---|---|---|
| 1 | Gazebo | `ros2 launch ur_slam_bringup ur5e_gazebo.launch.py` | `/clock` |
| 2 | MoveIt2 | `ros2 launch ur_slam_bringup ur5e_moveit.launch.py` | `/move_group/status` |
| 3 | RTAB-Map | `ros2 launch ur_slam_bringup ur5e_slam.launch.py` | `/rtabmap/mapData` |
| 4 | cloud_gate | `ros2 run ur_slam_tools cloud_gate.py` | `/cloud_gate/status` or トピック確認 |
| 5 | depth_image_noiser | `ros2 run ur_slam_tools depth_image_noiser.py` + NOISEパラメータ | `/camera/depth/depth/image_raw_noisy` |
| 6 | arc_sweep_jointtraj | `ros2 run ur_slam_tools arc_sweep_jointtraj.py` | プロセス終了まで待機 |
| 7 | coarse_to_fine_go_to_leaf5 | `ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5` + CONDITIONパラメータ | プロセス終了まで待機 |

### 起動待機の方針

- トピックが来るまで `ros2 topic hz <topic> --once` 等で待機
- タイムアウト（例: 60秒）を設けてタイムアウト時はエラー終了
- プロセス終了待ちは `wait $PID`

---

## CONDITION（経路計画方式）

| CONDITION | planning_mode | 説明 |
|---|---|---|
| A | `hybrid` | COARSE=OMPL + FINE=Cartesian |
| B | `ompl_two_stage` | COARSE=OMPL + FINE=OMPL |
| C | `ompl_direct` | OMPL直接（TODO-2実装後に有効化） |

### leaf5 固定パラメータ（全条件共通）

```bash
ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \
  -p use_sim_time:=true \
  -p world_frame:=base_link \
  -p ee_link:=tool0 \
  -p camera_frame:=camera_color_optical_frame \
  -p leaf_center_frame:=leaf_target \
  -p fine_goal_frame:=leaf_target_shot \
  -p view_goal_frame:=leaf_target_view \
  -p tf_timeout_sec:=10.0 \
  -p pre_back_m:=0.20 -p pre_down_m:=0.20 \
  -p coarse_max_tries:=5 -p coarse_retry_sleep_sec:=0.2 \
  -p coarse_planning_time:=15.0 -p fine_planning_time:=10.0 -p view_planning_time:=5.0 \
  -p coarse_vel_scale:=0.05 -p coarse_acc_scale:=0.05 \
  -p fine_vel_scale:=0.05 -p fine_acc_scale:=0.05 \
  -p view_vel_scale:=0.05 -p view_acc_scale:=0.05 \
  -p fine_orient_mode:=blend \
  -p fine_orient_blend_alpha:=0.6 \
  -p fine_cartesian_eef_step:=0.002 \
  -p fine_cartesian_jump_threshold:=0.0 \
  -p fine_cartesian_min_fraction:=0.60 \
  -p fine_cartesian_avoid_collisions:=true \
  -p fine_cartesian_eef_step_candidates:="[0.002, 0.005, 0.01]" \
  -p state_sync_sleep_ms:=300 \
  -p p0_warn_rad:=0.02 \
  -p csv_enable:=true \
  -p csv_path:=/home/fujino/ur5e_sim/results/leaf5_log.csv
```

### CONDITION別の切り替えパラメータ

```bash
# CONDITION A
-p planning_mode:=hybrid

# CONDITION B
-p planning_mode:=ompl_two_stage

# CONDITION C（TODO-2実装後）
-p planning_mode:=ompl_direct
```

### csv_run_idのフォーマット

```
<CONDITION>_<NOISE>_<3桁連番>
例: A_weak_001, B_strong_003
```

---

## NOISE（ノイズ条件）

### clean

```bash
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p seed:=42 \
  -p dropout_p:=0.0 \
  -p block_dropout:=false \
  -p gauss_a:=0.0 -p gauss_b:=0.0 \
  -p enable_edge_spikes:=false
```

### weak

```bash
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p seed:=42 \
  -p dropout_p:=0.01 \
  -p block_dropout:=true \
  -p block_count:=2 \
  -p block_radius_px:=12 \
  -p gauss_a:=0.003 \
  -p gauss_b:=0.0002 \
  -p enable_edge_spikes:=true \
  -p edge_grad_thr:=0.40 \
  -p edge_spike_p:=0.005 \
  -p edge_spike_min_factor:=0.97 \
  -p edge_spike_max_factor:=0.995
```

### medium

```bash
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p seed:=42 \
  -p dropout_p:=0.02 \
  -p block_dropout:=true \
  -p block_count:=5 \
  -p block_radius_px:=18 \
  -p gauss_a:=0.006 \
  -p gauss_b:=0.0003 \
  -p enable_edge_spikes:=true \
  -p edge_grad_thr:=0.35 \
  -p edge_spike_p:=0.015 \
  -p edge_spike_min_factor:=0.96 \
  -p edge_spike_max_factor:=0.99
```

### strong

```bash
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p seed:=42 \
  -p dropout_p:=0.05 \
  -p block_dropout:=true \
  -p block_count:=7 \
  -p block_radius_px:=20 \
  -p gauss_a:=0.010 \
  -p gauss_b:=0.0005 \
  -p enable_edge_spikes:=true \
  -p edge_grad_thr:=0.30 \
  -p edge_spike_p:=0.020 \
  -p edge_spike_min_factor:=0.93 \
  -p edge_spike_max_factor:=0.98
```

---

## 実装上の注意事項

- ワークスペースのsource忘れに注意：スクリプト冒頭で `source /home/fujino/ur5e_sim/install/setup.bash`
- 各試行の終了後は `kill` + `wait` で全子プロセスを確実に終了させる
- results/ ディレクトリが存在しない場合は自動作成する
- 試行開始・終了のログを標準出力に出す（例: `[TRIAL 1/10] START`）
- タイムアウト時はエラーログを出して次の試行へ進む（abort しない）
- 将来的に全条件（A/B/C × clean/weak/medium/strong × 10回）を回すモードを追加予定

---

## 実装予定ファイル

```
scripts/run_bench.sh
```
