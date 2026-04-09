# TODO

## 完了したこと

- [x] 環境構築（ur5e 関連パッケージのコピー・colcon build）
- [x] ドキュメント整理（CLAUDE.md + docs/）
- [x] 問題の原因分析（IK ブランチ跳び・345度問題）
- [x] moveit_kinematics.yaml を正しい場所に配置
- [x] CMakeLists.txt を実際のファイル配置に合わせて修正・ビルド確認済み
- [x] leaf1〜4 を `src/ur_slam_tools/src/_archive/` に移動済み（削除ではなくアーカイブ）
- [x] 不要ファイルの削除（go_to_leaf.cpp / pointcloud_noiser.py / scripts/ / leaf4_moveit_kinematics.yaml）
- [ ] TODO-1: run_bench.sh
- [ ] TODO-2: leaf5 条件C実装（seed_joints モード）
- [ ] TODO-3: analyze_bench.py

---

## TODO-1: `scripts/run_bench.sh`（自動実験スクリプト）

引数: `CONDITION`（A/B/C/D）, `NOISE`（clean/weak/mid/strong）, `N_TRIALS`（試行回数）

各試行でやること:
1. Gazebo のアームを home 姿勢にリセット
2. depth_image_noiser を NOISE に合わせた設定で起動
3. arc_sweep_jointtraj.py で首振りスキャン
4. coarse_to_fine_go_to_leaf5 を CONDITION に合わせたパラメータで実行
5. CSV に結果追記（csv_run_id に条件・ノイズ・試行番号を含める）
6. ノイザーを停止

---

## TODO-2: `coarse_to_fine_go_to_leaf5.cpp` への条件C用修正

`fine_orient_mode:="seed_joints"` を新たに追加:
- COARSE 到達後の関節状態を `std::vector<double>` で保存
- FINE フェーズで `move_group.setJointValueTarget()` を使いシードとして渡す
- 既存モードへの影響ゼロ・最小限の変更で実装

実装箇所:
1. COARSE 成功後に `move_group.getCurrentState()` で関節値を取得・保存
2. FINE フェーズで `fine_orient_mode == "seed_joints"` の分岐を追加
3. `setJointValueTarget` でゴールを関節空間で指定

---

## TODO-3: `scripts/analyze_bench.py`（結果集計スクリプト）

CSV の `detail` カラム（`plan_t=X exec_t=X joint_travel=X ...`）をパースして:
- 条件・ノイズ別に `exec_t`・`joint_travel`・`pos_err_m` を集計
- 箱ひげ図（matplotlib）と比較表を出力
- 成功率（FINE の status が SUCCESS の割合）も集計
