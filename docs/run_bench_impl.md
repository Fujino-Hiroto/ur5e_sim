# run_bench.sh 実装メモ

## 概要

`scripts/run_bench.sh` は、指定した CONDITION / NOISE 条件で実験を N 回連続自動実行するスクリプト。

```bash
./scripts/run_bench.sh [--dry-run] <CONDITION: A|B|C> <NOISE: clean|weak|medium|strong> [N_TRIALS=10]
```

## 起動順序

各試行で以下の順番にプロセスを起動し、実験完了後に全プロセスを終了する。

| # | プロセス | 方式 | 待機 |
|---|---------|------|------|
| 0 | TF publishers (4 nodes) | setsid + バックグラウンド | なし |
| 1 | Gazebo | setsid + バックグラウンド | 15s sleep + `/clock` 検証 |
| 2 | MoveIt2 | setsid + バックグラウンド | 15s sleep + `/monitored_planning_scene` 検証 |
| 2.5 | 床コリジョン追加 | `ros2 service call /apply_planning_scene` | 同期呼び出し |
| 3 | RTAB-Map | setsid + バックグラウンド | 20s sleep (検証スキップ) |
| 4 | depth_image_noiser | setsid + バックグラウンド | 5s sleep |
| 5 | cloud_gate | setsid + バックグラウンド | 5s sleep |
| 6 | arc_sweep_jointtraj | フォアグラウンド | wait + exit code 取得 |
| 7 | coarse_to_fine_go_to_leaf5 | フォアグラウンド | wait + exit code 取得 |

### 起動順序に関する注意

- **この順序は重要**。以前、cloud_gate / depth_image_noiser を RTAB-Map より前に移動したところ arc_sweep でアームが動かなくなった。手動実行時と同じ順序 (Gazebo → MoveIt → SLAM → noiser → gate → arc → leaf) にすること。
- RTAB-Map は `/rtabmap/mapData` の publish が遅いため readiness 検証をスキップしている。

## プロセス管理

### setsid + プロセスグループ kill

`ros2 launch` は内部で gzserver / gzclient 等を独立プロセスとして起動するため、PID 単体の追跡では子プロセスを殺せない。

- 全バックグラウンドプロセスを `setsid` で起動 → 新しいプロセスグループリーダーになる
- `GROUP_PIDS` 配列にグループ PID を保持
- cleanup で `kill -- -$gpid` によりグループ全体を一括終了
- SIGINT → 3 秒待ち → SIGKILL の段階的終了

### なぜ pkill を使わないか

以前 `pkill -f gzclient` を使っていたが、MoveIt のタイムアウト時に Gazebo GUI が巻き込まれる問題があった。プロセスグループ方式なら自分が起動したプロセスだけを確実に終了できる。

## readiness 検出

### sleep + topic 検証方式

```bash
wait_ready "Gazebo" 15 /clock
```

1. 固定 sleep で起動を待つ
2. `timeout 10 ros2 topic echo <topic> --once` で検証
3. 失敗しても WARN で続行可能（RTAB-Map 等）

### なぜ ros2 topic hz を使わないか

`set -o pipefail` 環境下で `ros2 topic hz | grep` のパイプを使うと、grep が成功しても ros2 topic hz が SIGPIPE (exit 141) で死に、pipefail によりパイプライン全体が失敗扱いになる。`ros2 topic echo --once` はパイプ不要でこの問題を回避できる。

## CONDITION / NOISE

### CONDITION

| CONDITION | planning_mode | fine_orient_mode | alpha |
|-----------|--------------|------------------|-------|
| A | ompl_two_stage | blend | 0.7 |
| B | ompl_two_stage | fixed | 0.7 |
| C | hybrid | blend | 0.7 |

### NOISE

clean / weak / medium / strong の 4 段階。depth_image_noiser のパラメータ (dropout_p, gauss_a/b, edge_spike 等) で制御。

## ログ

- 全行にタイムスタンプ `[HH:MM:SS]`
- `results/run_bench_YYYYMMDD_HHMMSS.log` に自動保存 (tee)
- dry-run 時はログファイルを生成しない
- 各試行の所要時間を記録
- 最後に `SUCCESS / FAIL / SKIP` の集計サマリー

### ステータスの意味

| ステータス | 意味 |
|-----------|------|
| SUCCESS | leaf5 が exit 0 で正常終了 |
| FAIL | arc_sweep または leaf5 が非ゼロで終了 |
| SKIP | インフラ起動失敗 (Gazebo, MoveIt 等のタイムアウト) |

## csv_run_id

`<CONDITION>_<NOISE>_<3桁連番>` 形式。例: `A_weak_001`, `B_strong_003`

## 環境セットアップ

スクリプト冒頭で以下を行う:

- `source install/setup.bash` (`set +u` / `set -u` で囲む — ROS の setup.bash が未定義変数を使うため)
- `GAZEBO_MODEL_PATH` の export (run_gazebo.sh と同等)
- `results/` ディレクトリの自動作成

## 床コリジョン

MoveIt の `/apply_planning_scene` サービス経由で 4m x 4m x 2cm のボックスを z=-0.1 に追加。`Rviz_floor.scene` の内容を再現している。

## チューニング可能な定数

```bash
GAZEBO_STARTUP_SEC=15   # Gazebo 起動待ち
MOVEIT_STARTUP_SEC=15   # MoveIt 起動待ち
SLAM_STARTUP_SEC=20     # RTAB-Map 起動待ち
TOOL_STARTUP_SEC=5      # cloud_gate / depth_image_noiser 起動待ち
CLEANUP_WAIT=10         # cleanup 後の待機
```

PC 性能に合わせて調整する。

## 開発中に遭遇した問題と解決策

| 問題 | 原因 | 解決 |
|------|------|------|
| GAZEBO_MODEL_PATH / TF publishers 欠落 | run_gazebo.sh からの移植漏れ | スクリプトに追加 |
| MoveIt 待機トピック名の誤り | `/move_group/status` → 存在しない | `/monitored_planning_scene` に修正 |
| pipefail で topic hz が常に失敗 | grep 成功 → SIGPIPE → pipefail で非ゼロ | `ros2 topic echo --once` に変更 |
| echo --once がトピック未存在で即失敗 | トピック advertise 前に実行 | sleep + リトライループ → sleep + 検証方式 |
| pkill が Gazebo GUI を誤殺 | 名前ベース kill が広すぎる | setsid + プロセスグループ kill |
| ros2 launch の子プロセスが PID 追跡外 | ExecuteProcess で独立プロセス生成 | setsid でグループ化 |
| 起動順序変更でアームが動かない | cloud_gate/noiser を SLAM 前に移動 | 手動実行時と同じ順序に戻す |
