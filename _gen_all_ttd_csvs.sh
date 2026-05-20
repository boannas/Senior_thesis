#!/usr/bin/env bash
# Generate ttd_across_seeds.csv for every per-genome rollout folder under the given roots.
# Idempotent: skips folders whose ttd_across_seeds.csv is newer than all their run_*.csv.
set -euo pipefail

ROOTS=(
  "StageRollouts_normal/initial"
  "StageRollouts_normal/transient"
  "StageRollouts_normal/emergent"
  "FinalGenomeRollouts_normal_seen"
  "FinalGenomeRollouts_unseen_food30"
  "FinalGenomeRollouts_unseen_hard"
)

cd "$(dirname "$0")"

for r in "${ROOTS[@]}"; do
  if [[ ! -d "$r" ]]; then
    echo "[skip] missing root: $r"
    continue
  fi
  echo "[root] $r"
  while IFS= read -r d; do
    n=$(find "$d" -maxdepth 1 -name 'run_*.csv' | wc -l)
    if [[ "$n" -eq 0 ]]; then
      continue
    fi
    if [[ -f "$d/ttd_across_seeds.csv" ]]; then
      # quick freshness check: skip if newer than every run_*.csv
      newest_run=$(ls -t "$d"/run_*.csv 2>/dev/null | head -n1)
      if [[ -n "$newest_run" && "$d/ttd_across_seeds.csv" -nt "$newest_run" ]]; then
        continue
      fi
    fi
    echo "  -> $d (n=$n)"
    python3 plot_runlog_ttd_across_seeds.py --runlog-dir "$d" >/dev/null
  done < <(find "$r" -mindepth 1 -maxdepth 1 -type d | sort)
done

echo "[ok] all ttd_across_seeds.csv refreshed"
