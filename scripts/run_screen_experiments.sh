#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_DIR="$REPO_ROOT/src/config"
RESULT_DIR="$REPO_ROOT/src/result_th1"
EXECUTABLE="$REPO_ROOT/build/amdahl"

if ! command -v screen >/dev/null 2>&1; then
  echo "GNU screen is not installed or not on PATH. Please install it before running this script." >&2
  exit 1
fi

if [[ ! -x "$EXECUTABLE" ]]; then
  echo "Executable $EXECUTABLE not found or not executable. Build the project before launching experiments." >&2
  exit 1
fi

if [[ ! -d "$CONFIG_DIR" ]]; then
  echo "Config directory $CONFIG_DIR does not exist." >&2
  exit 1
fi

mapfile -t config_files < <(find "$CONFIG_DIR" -maxdepth 1 -type f -name '*.json' | sort)
config_count=${#config_files[@]}

if (( config_count != 10 )); then
  echo "Expected exactly 10 JSON config files in $CONFIG_DIR, but found $config_count." >&2
  exit 1
fi

mkdir -p "$RESULT_DIR"

for config in "${config_files[@]}"; do
  session_name=$(basename "${config%.json}")
  log_file="$RESULT_DIR/${session_name}.log"
  screen -dmS "$session_name" bash -c "cd \"$REPO_ROOT\" && \"$EXECUTABLE\" \"$config\" > \"$log_file\" 2>&1"
  echo "Started screen session '$session_name' for config '$config'. Logs: $log_file"
done
