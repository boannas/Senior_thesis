#!/usr/bin/env bash
#
# Headless rollout batch for deep analysis.
# Runs tick-level logging (run_*.csv) using headless_rollout_log.py for three init conditions:
#   anti_maternal, random_uniform, pro_maternal
#
# Supports sharding across up to 12 terminals:
#   WORKER=0..11 ./run_headless_rollout_3inits.sh ...
# Each worker runs a disjoint subset of the 30 seeds per condition.
#
# Example (P1 genomes, normal world, 30 seeds each, 12 terminals):
#   export G_ANTI="Emergence_results/normal/P1_anti_maternal_seed42/final_genome.json"
#   export G_RAND="Emergence_results/normal/P1_random_uniform_seed42/final_genome.json"
#   export G_PRO="Emergence_results/normal/P1_pro_maternal_seed42/final_genome.json"
#   for w in $(seq 0 11); do
#     WORKER=$w ./run_headless_rollout_3inits.sh --world normal --plasticity-eval p1 --out-root DeepRollouts_normal_P1 &
#   done
#   wait
#
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_headless_rollout_3inits.sh [options]

Genomes (required; can be env vars or flags):
  --genome-anti PATH   (or env G_ANTI)
  --genome-rand PATH   (or env G_RAND)
  --genome-pro  PATH   (or env G_PRO)

World/config (defaults aligned with RUN_GUIDE normal runs):
  --world easy|normal|hard   (default normal; sets threats 0/1/2)
  --max-ticks N              (default 1000)
  --grid-w N                 (default 15)
  --grid-h N                 (default 15)
  --food-spawn-interval N    (default 15)
  --food-spawn-n N           (default 1)

Rollout count:
  --n-seeds N                (default 30)  number of rollouts per condition
  --seed-start N             (default 10000) first seed; uses seed-start..seed-start+n-seeds-1

Plasticity during rollout:
  --plasticity-eval none|p1|p2  (default none)

Outputs:
  --out-root DIR             (default DeepRollouts_<world>_<plasticity>)
  Writes:
    <out-root>/anti_maternal/run_<seed>.csv
    <out-root>/random_uniform/run_<seed>.csv
    <out-root>/pro_maternal/run_<seed>.csv

Sharding across 12 terminals:
  Set WORKER=0..11 (optional). Each worker runs ~ceil(n_seeds/12) seeds per condition.
EOF
}

WORLD="normal"
MAX_TICKS="1000"
GRID_W="15"
GRID_H="15"
FOOD_INT="15"
FOOD_N="1"
N_SEEDS="30"
SEED_START="10000"
PLASTICITY_EVAL="none"
OUT_ROOT=""

G_ANTI="${G_ANTI:-}"
G_RAND="${G_RAND:-}"
G_PRO="${G_PRO:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --genome-anti) G_ANTI="${2:-}"; shift 2 ;;
    --genome-rand) G_RAND="${2:-}"; shift 2 ;;
    --genome-pro)  G_PRO="${2:-}"; shift 2 ;;
    --world) WORLD="${2:-}"; shift 2 ;;
    --max-ticks) MAX_TICKS="${2:-}"; shift 2 ;;
    --grid-w) GRID_W="${2:-}"; shift 2 ;;
    --grid-h) GRID_H="${2:-}"; shift 2 ;;
    --food-spawn-interval) FOOD_INT="${2:-}"; shift 2 ;;
    --food-spawn-n) FOOD_N="${2:-}"; shift 2 ;;
    --n-seeds) N_SEEDS="${2:-}"; shift 2 ;;
    --seed-start) SEED_START="${2:-}"; shift 2 ;;
    --plasticity-eval) PLASTICITY_EVAL="${2:-}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$G_ANTI" || -z "$G_RAND" || -z "$G_PRO" ]]; then
  echo "[error] need all three genomes: --genome-anti/--genome-rand/--genome-pro (or G_ANTI/G_RAND/G_PRO env vars)" >&2
  exit 2
fi

case "$WORLD" in
  easy) THR=0 ;;
  normal) THR=1 ;;
  hard) THR=2 ;;
  *) echo "[error] --world must be easy|normal|hard, got: $WORLD" >&2; exit 2 ;;
esac

case "$PLASTICITY_EVAL" in
  none) PLAST_ARGS=(--plasticity none) ;;
  p1) PLAST_ARGS=(--plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick) ;;
  p2) PLAST_ARGS=(--plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick) ;;
  *) echo "[error] --plasticity-eval must be none|p1|p2, got: $PLASTICITY_EVAL" >&2; exit 2 ;;
esac

ROOT="$(cd "$(dirname "$0")" && pwd)"
if [[ -z "$OUT_ROOT" ]]; then
  OUT_ROOT="DeepRollouts_${WORLD}_${PLASTICITY_EVAL}"
fi
OUT_DIR="$ROOT/$OUT_ROOT"
mkdir -p "$OUT_DIR"/{anti_maternal,random_uniform,pro_maternal}

TOTAL_WORKERS=12
WORKER="${WORKER:-}"
if [[ -n "$WORKER" ]]; then
  if ! [[ "$WORKER" =~ ^[0-9]+$ ]] || (( WORKER < 0 || WORKER >= TOTAL_WORKERS )); then
    echo "[error] WORKER must be 0..$((TOTAL_WORKERS-1)), got: $WORKER" >&2
    exit 2
  fi
fi

N_TOTAL=$((N_SEEDS))
CHUNK=$(( (N_TOTAL + TOTAL_WORKERS - 1) / TOTAL_WORKERS ))  # ceil
START_OFF=0
if [[ -n "$WORKER" ]]; then
  START_OFF=$(( WORKER * CHUNK ))
fi
RUN_N=$(( N_TOTAL - START_OFF ))
if (( RUN_N < 0 )); then RUN_N=0; fi
if (( RUN_N > CHUNK )); then RUN_N=$CHUNK; fi
RUN_SEED=$(( SEED_START + START_OFF ))

if (( RUN_N == 0 )); then
  echo "[info] WORKER=${WORKER:-all} has no seeds to run (n_seeds=$N_TOTAL)."
  exit 0
fi

echo "[info] world=$WORLD thr=$THR plasticity_eval=$PLASTICITY_EVAL"
echo "[info] seeds: start=$RUN_SEED  n=$RUN_N  (of total $N_TOTAL; worker=${WORKER:-all})"
echo "[info] out: $OUT_DIR"

run_one() {
  local label=$1 genome=$2 out_sub=$3
  echo "[run] $label -> $out_sub  genome=$(basename "$genome")"
  python3 headless_rollout_log.py \
    --genome "$genome" \
    --seed "$RUN_SEED" --n-seeds "$RUN_N" \
    --out-dir "$out_sub" \
    --max-ticks "$MAX_TICKS" \
    --grid-w "$GRID_W" --grid-h "$GRID_H" --threats "$THR" \
    --food-spawn-interval "$FOOD_INT" --food-spawn-n "$FOOD_N" \
    "${PLAST_ARGS[@]}"
}

run_one "anti_maternal" "$G_ANTI" "$OUT_DIR/anti_maternal"
run_one "random_uniform" "$G_RAND" "$OUT_DIR/random_uniform"
run_one "pro_maternal" "$G_PRO" "$OUT_DIR/pro_maternal"

echo "[ok] finished worker=${WORKER:-all}"

