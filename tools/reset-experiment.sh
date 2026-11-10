#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dry_run=false

if [[ ${1:-} == "--dry-run" ]]; then
  dry_run=true
elif [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--dry-run]" >&2
  exit 2
fi

log_files=(
  "$repo_root/tools/learning-results-grabbing/learning_results_log.txt"
  "$repo_root/tools/learning-visualization-server/received_data.log"
)

echo "Stop the middleware and visualization server before resetting."

for log_file in "${log_files[@]}"; do
  if [[ ! -e $log_file ]]; then
    echo "Already absent: ${log_file#"$repo_root/"}"
    continue
  fi

  if $dry_run; then
    echo "Would clear: ${log_file#"$repo_root/"}"
  else
    truncate --size 0 -- "$log_file"
    echo "Cleared: ${log_file#"$repo_root/"}"
  fi
done

if ! $dry_run; then
  echo "Experiment logs reset. Restart the server and middleware."
fi
