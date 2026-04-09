# プロジェクト概要

## 研究テーマ
UR5e アームに搭載した RGB-D カメラで植物の葉に Coarse-to-Fine 動作計画で近づき撮影する。
深度センサにノイズが加わった環境下での動作性能（所要時間・到達精度）を評価する。

## システム構成

```
Gazebo シミュレータ
  UR5e + RGB-D カメラ（ur5e_with_camera.urdf.xacro）
  植物モデル（abstract_plant）

depth_image_noiser.py
  /camera/depth/depth/image_raw
  → /camera/depth/depth/image_raw_noisy  ← SLAM に直接入力される

RTAB-Map（ur5e_slam.launch.py）
  RGB + Depth(noisy) → SLAM → 葉の TF を継続更新

arc_sweep_jointtraj.py  → 首振りで環境スキャン（地図構築フェーズ）
cloud_gate.py           → 首振り中のみ地図更新を許可（比較条件を揃えるため）

coarse_to_fine_go_to_leaf5  → 葉へのアプローチ動作・結果を CSV に記録
```

## TF フレーム構成

```
world (= base_link)
  └── tool0 (ee_link)
      └── camera_color_optical_frame (camera_frame)

plant_base         ← 植物モデルの根元
  └── leaf_target        ← 葉の中心
      └── leaf_target_shot   ← 最終撮影地点（fine_goal_frame）
          └── leaf_target_view   ← 姿勢合わせ地点（view_goal_frame）
```

TF は `scripts/run_gazebo.sh` の static_transform_publisher で定義。

## リポジトリ構成（整理後）

```
src/
  ur_slam_bringup/    Gazebo 環境・MoveIt 起動・植物モデル
  ur_slam_tools/      実験ノード・ノイザー
    src/
      coarse_to_fine_go_to_leaf5.cpp  ★ メインノード
      depth_image_noiser.py           ★ ノイザー
      arc_sweep_jointtraj.py          首振りスキャン
      cloud_gate.py                   地図更新ゲート
      pointcloud_metrics_csv.py       点群メトリクス記録
  Universal_Robots_ROS2_Description   カスタマイズ済み（clone 不可）
  Universal_Robots_ROS2_Driver        カスタマイズ済み（clone 不可）
  Universal_Robots_ROS2_Gazebo_Simulation  カスタマイズ済み（clone 不可）

scripts/
  build.sh              colcon build ラッパー
  bootstrap.sh          rosdep install
  run_gazebo.sh         Gazebo + TF 起動
  check_sim.sh          TF・カメラトピック疎通確認
  spawn_plant_collision.py  MoveIt PlanningScene に植物を登録
  monitor_metrics.py    衝突・視線角・距離のリアルタイム記録

docs/               ドキュメント群
results/            CSV 出力先
bags/               実験データ（参照用）
runs/               実験ログ（参照用）
Rviz_floor.scene    床の collision scene（床抜け防止）
```

## ノイズ強度（depth_image_noiser）

| 強度 | dropout_p | gauss_a | gauss_b | edge_spikes | flicker |
|---|---|---|---|---|---|
| clean | 0 | 0 | 0 | false | false |
| weak | 0.01 | 0.0015 | 0.0006 | true | false |
| mid | 0.03 | 0.0030 | 0.0012 | true | true |
| strong | 0.07 | 0.0060 | 0.0025 | true | true |
