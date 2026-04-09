# 実験ガイド

## 環境構築

```bash
# 1. rosdep
./scripts/bootstrap.sh

# 2. ビルド
./scripts/build.sh

# 3. source（毎ターミナル）
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 実験の起動手順（ターミナル4つ）

```bash
# [T1] Gazebo + TF
./scripts/run_gazebo.sh

# [T2] MoveIt
ros2 launch ur_slam_bringup ur5e_moveit.launch.py

# [T3] SLAM
ros2 launch ur_slam_bringup ur5e_slam.launch.py with_gazebo:=false

# [T4] ノイザー起動（mid ノイズ例）
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p dropout_p:=0.03 \
  -p block_dropout:=true -p block_count:=8 -p block_radius_px:=6 \
  -p gauss_a:=0.0030 -p gauss_b:=0.0012 \
  -p enable_edge_spikes:=true \
  -p enable_flicker:=true -p flicker_hz:=1.0 -p flicker_amp:=0.6
```

## 実験実行順序

```bash
# 1. 植物を MoveIt PlanningScene に登録（別ターミナル）
python3 scripts/spawn_plant_collision.py

# 2. 首振りスキャン（地図構築）
ros2 run ur_slam_tools arc_sweep_jointtraj.py

# 3. leaf5 実行
ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \
  -p planning_mode:=hybrid \
  [... その他パラメータ ...]
```

## ノイズ強度プリセット（depth_image_noiser パラメータ）

### clean（ノイズなし）
ノイザー起動不要。

### weak
```
dropout_p:=0.01 block_count:=4 block_radius_px:=4
gauss_a:=0.0015 gauss_b:=0.0006
enable_edge_spikes:=true enable_flicker:=false
```

### mid
```
dropout_p:=0.03 block_count:=8 block_radius_px:=6
gauss_a:=0.0030 gauss_b:=0.0012
enable_edge_spikes:=true enable_flicker:=true flicker_hz:=1.0 flicker_amp:=0.6
```

### strong
```
dropout_p:=0.07 block_count:=14 block_radius_px:=8
gauss_a:=0.0060 gauss_b:=0.0025
enable_edge_spikes:=true enable_flicker:=true flicker_hz:=1.2 flicker_amp:=0.9
frame_drop_p:=0.02
```

## 検証実験（IK ブランチ問題の切り分け）

### 実験条件

| 条件 | planning_mode | fine_orient_mode | 検証対象 |
|---|---|---|---|
| A（卒論ベース） | ompl_two_stage | blend α=0.7 | 問題①②あり |
| B | ompl_two_stage | fixed | 問題①を抑制 |
| C | ompl_two_stage | seed_joints | 問題②を抑制（未実装）|
| D | hybrid | blend α=0.7 | 両方解決 |

### フェーズ1（素早い確認）
条件A vs D × midノイズ × 各10回 → `scripts/run_bench.sh`（未作成）で自動実行

### フェーズ2（本格検証）
全4条件 × 4ノイズ × 20回

## 結果の確認

```bash
python3 scripts/analyze_bench.py results/leaf5_log.csv
```
（`analyze_bench.py` は未作成 → `docs/todo.md` 参照）
