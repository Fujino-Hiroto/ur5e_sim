# 実装ルール

Claude Code が `scripts/` 以下のスクリプトを実装する際に従うべきルール。

---

## 1. 環境・パス情報

- ワークスペースのルート：`/home/fujino/ur5e_sim`
- ROS2のsource：スクリプト冒頭で必ず以下を実行すること
  ```bash
  source /home/fujino/ur5e_sim/install/setup.bash
  ```
- 実験結果の出力先：`/home/fujino/ur5e_sim/results/`
  - スクリプト内で `mkdir -p results/` を実行してディレクトリを自動作成すること

---

## 2. シェルスクリプトのコーディング規約

- スクリプト冒頭に必ず記述する：
  ```bash
  #!/bin/bash
  set -uo pipefail
  ```
- `set -e`（エラーで即停止）は**使用しない**
  - 理由：実験スクリプトでは衝突・プランニング失敗などのエラーが発生しても次の試行へ継続する必要があるため
  - エラーハンドリングは各コマンドの終了コードを個別にチェックする方式で実装する
- エラーが致命的（ワークスペースが存在しない・必須引数が未指定など）な場合のみ `exit 1` で即停止する
- ログは以下のフォーマットで標準出力へ出力する：
  ```
  [TRIAL 1/10] START  condition=A  noise=weak
  [TRIAL 1/10] DONE   status=SUCCESS
  [TRIAL 2/10] START  condition=A  noise=weak
  [TRIAL 2/10] DONE   status=FAIL (exit code: 1)
  ```

---

## 3. ROSに関するルール

- **トピック待機**：`ros2 topic hz <topic> --once` でトピックが来るまで待機する
  - タイムアウトは各プロセスで60秒を基本とする
  - タイムアウト時はエラーログを出して `return 1`（試行を失敗扱いにして継続）
- **プロセスの終了**：ROSプロセスは `kill -INT $PID` で終了させる（`SIGINT` = Ctrl+C相当）
  - 終了後は必ず `wait $PID` でプロセスの完全終了を待つ
  - タイムアウト（10秒）後も終了しない場合は `kill -KILL $PID` で強制終了する
- `use_sim_time:=true` を全ROSノードに必ず設定すること
- 全プロセスのPIDを変数に保持し、スクリプト終了時（正常・異常問わず）に `trap` で必ずkillすること：
  ```bash
  trap cleanup EXIT
  cleanup() {
    kill -INT $GAZEBO_PID $MOVEIT_PID ... 2>/dev/null
    wait
  }
  ```

---

## 4. dry-runオプション

- 全ての実験スクリプトに `--dry-run` オプションを実装すること
- `--dry-run` 指定時は実際のROSコマンドを実行せず、実行予定のコマンドを標準出力に表示するだけにする
- 実装例：
  ```bash
  DRY_RUN=false
  if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
  fi

  run_cmd() {
    if $DRY_RUN; then
      echo "[DRY-RUN] $*"
    else
      "$@"
    fi
  }
  ```
