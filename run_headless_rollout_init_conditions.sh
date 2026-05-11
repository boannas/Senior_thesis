#!/usr/bin/env bash
#
# Headless rollout batch from *initialization conditions* (anti/random/pro),
# not from evolved final genomes.
#
# For each seed S in a range, we:
#   1) generate an init genome using run_evolve_lineage.initialize_genome(init_mode, init_seed=S, init_noise=...)
#   2) run one headless rollout with episode seed S (tick-level run_log CSV)
#
# Supports sharding across up to 12 terminals:
#   WORKER=0..11 ./run_headless_rollout_init_conditions.sh ...
#
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_headless_rollout_init_conditions.sh [options]

World/config (defaults aligned with RUN_GUIDE normal runs):
  --world easy|normal|hard   (default normal; sets threats 0/1/2)
  --max-ticks N              (default 1000)
  --grid-w N                 (default 15)
  --grid-h N                 (default 15)
  --food-spawn-interval N    (default 15)
  --food-spawn-n N           (default 1)

Seeds:
  --n-seeds N                (default 30)  number of seeds per condition
  --seed-start N             (default 10000)  seeds S..S+N-1 used for BOTH init_seed and rollout seed

Initialization:
  --init-noise X             (default 0.05) Gaussian jitter applied to template modes
                              (baseline_zero / anti_maternal / pro_maternal / from_json).
                              Note: random_uniform ignores init_noise (matches run_evolve_lineage).
  --init-low X               (default 0.05) lower bound for random_uniform weights
  --init-high X              (default 1.0)  upper bound for random_uniform weights

Plasticity during rollout:
  --plasticity-eval none|p1|p2  (default none)

Outputs:
  --out-root DIR             (default InitRollouts_<world>_<plasticity>)
  Writes:
    <out-root>/genomes/<init>/seed<S>.json           (generated init genomes)
    <out-root>/<init>/run_<seed>.csv                (tick-level logs)
    where <init> is anti_maternal / random_uniform / pro_maternal

Sharding across 12 terminals:
  Set WORKER=0..11 (optional). Each worker runs a disjoint subset of seeds.
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
INIT_NOISE="0.05"
INIT_LOW="0.05"
INIT_HIGH="1.0"
PLASTICITY_EVAL="none"
OUT_ROOT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD="${2:-}"; shift 2 ;;
    --max-ticks) MAX_TICKS="${2:-}"; shift 2 ;;
    --grid-w) GRID_W="${2:-}"; shift 2 ;;
    --grid-h) GRID_H="${2:-}"; shift 2 ;;
    --food-spawn-interval) FOOD_INT="${2:-}"; shift 2 ;;
    --food-spawn-n) FOOD_N="${2:-}"; shift 2 ;;
    --n-seeds) N_SEEDS="${2:-}"; shift 2 ;;
    --seed-start) SEED_START="${2:-}"; shift 2 ;;
    --init-noise) INIT_NOISE="${2:-}"; shift 2 ;;
    --init-low) INIT_LOW="${2:-}"; shift 2 ;;
    --init-high) INIT_HIGH="${2:-}"; shift 2 ;;
    --plasticity-eval) PLASTICITY_EVAL="${2:-}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

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
  OUT_ROOT="InitRollouts_${WORLD}_${PLASTICITY_EVAL}"
fi
if [[ "$OUT_ROOT" == /* ]]; then
  OUT_DIR="$OUT_ROOT"
else
  OUT_DIR="$ROOT/$OUT_ROOT"
fi

mkdir -p "$OUT_DIR"/{anti_maternal,random_uniform,pro_maternal}
mkdir -p "$OUT_DIR/genomes"/{anti_maternal,random_uniform,pro_maternal}

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
RUN_SEED_START=$(( SEED_START + START_OFF ))

if (( RUN_N == 0 )); then
  echo "[info] WORKER=${WORKER:-all} has no seeds to run (n_seeds=$N_TOTAL)."
  exit 0
fi

echo "[info] world=$WORLD thr=$THR plasticity_eval=$PLASTICITY_EVAL"
echo "[info] init_noise=$INIT_NOISE  random_uniform=[${INIT_LOW},${INIT_HIGH}]"
echo "[info] seeds: start=$RUN_SEED_START  n=$RUN_N  (of total $N_TOTAL; worker=${WORKER:-all})"
echo "[info] out: $OUT_DIR"

gen_genome() {
  local init_mode=$1 seed=$2 out_json=$3
  python3 - "$init_mode" "$seed" "$out_json" "$INIT_NOISE" "$INIT_LOW" "$INIT_HIGH" <<'PY'
import json
import sys
from run_evolve_lineage import initialize_genome

init_mode = sys.argv[1]
seed = int(sys.argv[2])
out_json = sys.argv[3]
init_noise = float(sys.argv[4])
init_low = float(sys.argv[5])
init_high = float(sys.argv[6])

g = initialize_genome(
    init_mode=init_mode,
    init_seed=seed,
    init_noise=init_noise,
    init_low=init_low,
    init_high=init_high,
)

with open(out_json, "w") as f:
    json.dump(g, f, indent=2)
PY
}

run_rollout() {
  local init=$1 seed=$2 genome_json=$3 out_sub=$4
  python3 headless_rollout_log.py \
    --genome "$genome_json" \
    --seed "$seed" --n-seeds 1 \
    --csv "$out_sub/run_${seed}.csv" \
    --max-ticks "$MAX_TICKS" \
    --grid-w "$GRID_W" --grid-h "$GRID_H" --threats "$THR" \
    --food-spawn-interval "$FOOD_INT" --food-spawn-n "$FOOD_N" \
    "${PLAST_ARGS[@]}"
}

for ((k=0; k<RUN_N; k++)); do
  s=$(( RUN_SEED_START + k ))
  for init in anti_maternal random_uniform pro_maternal; do
    gj="$OUT_DIR/genomes/$init/seed${s}.json"
    if [[ ! -f "$gj" ]]; then
      echo "[genome] init=$init seed=$s -> $gj"
      gen_genome "$init" "$s" "$gj"
    fi
    echo "[rollout] init=$init seed=$s"
    run_rollout "$init" "$s" "$gj" "$OUT_DIR/$init"
  done
done

echo "[ok] finished worker=${WORKER:-all}"

