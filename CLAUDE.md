# CLAUDE.md

ROS 2 Humble + Gazebo + MoveIt2 + RTAB-Map による
UR5e アームの植物葉への Coarse-to-Fine アプローチ実験リポジトリ。

## 詳細ドキュメント（必要に応じて参照）

| 状況 | 読むファイル |
|---|---|
| プロジェクト全体・研究背景を把握したい | `docs/project_overview.md` |
| メインノード leaf5 のパラメータや動作を確認したい | `docs/node_leaf5.md` |
| 実験を実行・再現したい | `docs/experiment_guide.md` |
| 次に実装すべきことを確認したい | `docs/todo.md` |
| スクリプトを実装・修正したい | `docs/implementation_rules.md` |
| run_bench.sh の仕様を確認したい | `docs/run_bench_spec.md` |

## 重要な注意事項

- **実験で使うノイザーは `depth_image_noiser.py`**
- **メインノードは `coarse_to_fine_go_to_leaf5`**（leaf1〜4 は `src/ur_slam_tools/src/_archive/` に移動済み）
- **`use_sim_time:=true` を必ず設定すること**
- ur5e 関連パッケージ（`Universal_Robots_ROS2_*`）はカスタマイズ済みのため
  clone でなく元の研究ワークスペースからコピーすること
