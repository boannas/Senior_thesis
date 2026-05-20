#!/usr/bin/env bash
#
# Headless rollout batch for *final genomes* from emergence experiments.
#
# For each final_genome.json under an emergence root, run N rollouts (N "worlds")
# with tick-level logging using headless_rollout_log.py.
#
# Plasticity rule can be inferred from the run folder name:
#   E2_* -> plasticity none
#   P1_* -> plasticity p1 (outcome_adaptive + global)
#   P2_* -> plasticity p2 (outcome + local)
#
# Supports sharding across up to 12 terminals by splitting the genome list:
#   WORKER=0..11 ./run_headless_rollout_emergence_final_genomes.sh ...
#
set -euo pipefail
shopt -s nullglob

usage() {
  cat <<'EOF'
Usage:
  run_headless_rollout_emergence_final_genomes.sh [options]

Inputs:
  --genome-glob "<glob>"   (required) glob for genome JSON files.
                           Example: "Emergence_results/normal/*/final_genome.json"
                           Checkpoint snapshots (plasticity inferred from parent run dir):
                           "Emergence_results/normal/*/checkpoints/best_gen0000.json"

World/config:
  --world easy|normal|hard    (default normal; sets threats 0/1/2)
  --max-ticks N               (default 1000)
  --grid-w N                  (default 15)
  --grid-h N                  (default 15)
  --food-spawn-interval N     (default 15)
  --food-spawn-n N            (default 1)

Rollouts ("worlds"):
  --n-worlds N                (default 30) number of rollout seeds per genome
  --seed-start N              (default 20000) first rollout seed (S..S+N-1)

Plasticity during rollout:
  --plasticity-eval infer|none|p1|p2   (default infer)
    infer: decide from run dir prefix (E2_/P1_/P2_)
    none : force --plasticity none for all genomes
    p1   : force P1 rule for all genomes
    p2   : force P2 rule for all genomes

  --ablate-self-motivation-fear
    Passes --ablate-self-motivation-fear to headless_rollout_log.py (fear excluded from
    Self motivation blend only; Forage/Protect unchanged).

Outputs:
  --out-root DIR              (default FinalGenomeRollouts_<world>)
  Writes one folder per genome:
    <out-root>/<run_dir>/run_<seed>.csv
  where <run_dir> is the parent folder name (e.g. P1_anti_maternal_seed42).

Sharding across 12 terminals:
  Set WORKER=0..11 (optional). Each worker runs a disjoint subset of genomes.

Examples:
  # One terminal, all genomes, 30 worlds each:
  ./run_headless_rollout_emergence_final_genomes.sh \
    --genome-glob "Emergence_results/normal/*/final_genome.json" \
    --world normal --n-worlds 30 --seed-start 20000

  # 12 terminals:
  WORKER=0  ./run_headless_rollout_emergence_final_genomes.sh --genome-glob "Emergence_results/normal/*/final_genome.json"
  ...
  WORKER=11 ./run_headless_rollout_emergence_final_genomes.sh --genome-glob "Emergence_results/normal/*/final_genome.json"
EOF
}

GENOME_GLOB=""
WORLD="normal"
MAX_TICKS="1000"
GRID_W="15"
GRID_H="15"
FOOD_INT="15"
FOOD_N="1"
N_WORLDS="30"
SEED_START="20000"
PLASTICITY_EVAL="infer"
OUT_ROOT=""
ABLATE_SELF_FEAR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --genome-glob) GENOME_GLOB="${2:-}"; shift 2 ;;
    --world) WORLD="${2:-}"; shift 2 ;;
    --max-ticks) MAX_TICKS="${2:-}"; shift 2 ;;
    --grid-w) GRID_W="${2:-}"; shift 2 ;;
    --grid-h) GRID_H="${2:-}"; shift 2 ;;
    --food-spawn-interval) FOOD_INT="${2:-}"; shift 2 ;;
    --food-spawn-n) FOOD_N="${2:-}"; shift 2 ;;
    --n-worlds) N_WORLDS="${2:-}"; shift 2 ;;
    --seed-start) SEED_START="${2:-}"; shift 2 ;;
    --plasticity-eval) PLASTICITY_EVAL="${2:-}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:-}"; shift 2 ;;
    --ablate-self-motivation-fear) ABLATE_SELF_FEAR="1"; shift 1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$GENOME_GLOB" ]]; then
  echo "[error] --genome-glob is required" >&2
  usage
  exit 2
fi

case "$WORLD" in
  easy) THR=0 ;;
  normal) THR=1 ;;
  hard) THR=2 ;;
  *) echo "[error] --world must be easy|normal|hard, got: $WORLD" >&2; exit 2 ;;
esac

case "$PLASTICITY_EVAL" in
  infer|none|p1|p2) ;;
  *) echo "[error] --plasticity-eval must be infer|none|p1|p2, got: $PLASTICITY_EVAL" >&2; exit 2 ;;
esac

ROOT="$(cd "$(dirname "$0")" && pwd)"
if [[ -z "$OUT_ROOT" ]]; then
  OUT_ROOT="FinalGenomeRollouts_${WORLD}"
fi
if [[ "$OUT_ROOT" == /* ]]; then
  OUT_DIR="$OUT_ROOT"
else
  OUT_DIR="$ROOT/$OUT_ROOT"
fi
mkdir -p "$OUT_DIR"

genomes=( $GENOME_GLOB )
if [[ ${#genomes[@]} -eq 0 ]]; then
  echo "[error] glob matched 0 files: $GENOME_GLOB" >&2
  exit 2
fi

TOTAL_WORKERS=12
WORKER="${WORKER:-}"
if [[ -n "$WORKER" ]]; then
  if ! [[ "$WORKER" =~ ^[0-9]+$ ]] || (( WORKER < 0 || WORKER >= TOTAL_WORKERS )); then
    echo "[error] WORKER must be 0..$((TOTAL_WORKERS-1)), got: $WORKER" >&2
    exit 2
  fi
fi

# Shard by genome index (each worker runs a disjoint subset of genomes).
my_genomes=()
for i in "${!genomes[@]}"; do
  if [[ -z "$WORKER" ]] || (( i % TOTAL_WORKERS == WORKER )); then
    my_genomes+=( "${genomes[$i]}" )
  fi
done

echo "[info] world=$WORLD thr=$THR n_worlds=$N_WORLDS seed_start=$SEED_START"
echo "[info] plasticity_eval=$PLASTICITY_EVAL"
echo "[info] ablate_self_motivation_fear=${ABLATE_SELF_FEAR:-0}"
echo "[info] genomes: total=${#genomes[@]}  this_worker=${#my_genomes[@]}  worker=${WORKER:-all}"
echo "[info] out: $OUT_DIR"

# Parent of genome path is usually the emergence run dir (E2_*_seedN).
# If genome lives under .../<run_dir>/checkpoints/best_genXXXX.json, use <run_dir>.
run_dir_for_genome() {
  local g=$1
  local d base
  d="$(dirname "$g")"
  base="$(basename "$d")"
  if [[ "${base,,}" == "checkpoints" ]]; then
    basename "$(dirname "$d")"
  else
    printf '%s\n' "$base"
  fi
}

plast_args_for_run_dir() {
  local run_dir=$1
  case "$PLASTICITY_EVAL" in
    none) echo "--plasticity none" ;;
    p1)   echo "--plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick" ;;
    p2)   echo "--plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick" ;;
    infer)
      if [[ "$run_dir" == E2_* ]]; then
        echo "--plasticity none"
      elif [[ "$run_dir" == P1_* ]]; then
        echo "--plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick"
      elif [[ "$run_dir" == P2_* ]]; then
        echo "--plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick"
      else
        echo "[error] cannot infer plasticity from run_dir=$run_dir (expected E2_/P1_/P2_ prefix)" >&2
        return 2
      fi
      ;;
  esac
}

for g in "${my_genomes[@]}"; do
  if [[ ! -f "$g" ]]; then
    echo "[warn] missing file: $g" >&2
    continue
  fi
  run_dir="$(run_dir_for_genome "$g")"
  out_sub="$OUT_DIR/$run_dir"
  mkdir -p "$out_sub"

  plast_args="$(plast_args_for_run_dir "$run_dir")"
  if [[ "$plast_args" == \[error\]* ]]; then
    echo "$plast_args" >&2
    exit 2
  fi

  extra_ablate=()
  if [[ -n "$ABLATE_SELF_FEAR" ]]; then
    extra_ablate=(--ablate-self-motivation-fear)
  fi

  echo "[run] $run_dir  worlds=$N_WORLDS  seed_start=$SEED_START"
  # shellcheck disable=SC2086
  python3 headless_rollout_log.py \
    --genome "$g" \
    --seed "$SEED_START" --n-seeds "$N_WORLDS" \
    --out-dir "$out_sub" \
    --max-ticks "$MAX_TICKS" \
    --grid-w "$GRID_W" --grid-h "$GRID_H" --threats "$THR" \
    --food-spawn-interval "$FOOD_INT" --food-spawn-n "$FOOD_N" \
    $plast_args \
    "${extra_ablate[@]}"
done

echo "[ok] finished worker=${WORKER:-all}"

