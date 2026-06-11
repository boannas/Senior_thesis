#!/usr/bin/env bash
#
# Evaluate saved final genomes for mother/child TTD over many episodes.
# Produces one output folder per genome containing lineage_generations.csv (1 row: generation 0).
#
# Example (evaluate all emergence final genomes under normal-world config):
#   ./eval_final_genomes_ttd.sh \
#     --genome-glob "Emergence_results/normal/*/final_genome.json" \
#     --world normal --episodes 128 --seed-master 123
#
# Then extract TTD quickly:
#   python3 - <<'PY'
#   import glob, pandas as pd
#   for p in sorted(glob.glob("FinalGenome_TTD_eval/normal/*/lineage_generations.csv")):
#       r = pd.read_csv(p).iloc[-1]
#       print(p, "child", r.get("mean_child_ttd_norm"), "mother", r.get("mean_mother_ttd_norm"))
#   PY
#
set -euo pipefail
shopt -s nullglob

usage() {
  cat <<'EOF'
Usage:
  eval_final_genomes_ttd.sh --genome-glob "<glob>" [options]

Required:
  --genome-glob   Glob for final_genome.json files (quoted).

Options (defaults match RUN_GUIDE emergence/sweep normal config):
  --world               easy|normal|hard   (default: normal)
  --episodes            N                  (default: 128)
  --seed-master         N                  (default: 123)
  --max-ticks           N                  (default: 1000)
  --grid-w              N                  (default: 15)
  --grid-h              N                  (default: 15)
  --food-spawn-interval N                  (default: 15)
  --food-spawn-n        N                  (default: 1)
  --alpha-child         X                  (default: 0.5)
  --fitness-mode        MODE               (default: ttd_overall)

Plasticity during evaluation:
  --plasticity-eval  none|p1|p2   (default: none)
    none: --plasticity none
    p1  : --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick
    p2  : --plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick

Output:
  FinalGenome_TTD_eval/<world>/<tag>/
  where <tag> is derived from the genome's parent folder name.
EOF
}

WORLD="normal"
EPISODES="128"
SEED_MASTER="123"
MAX_TICKS="1000"
GRID_W="15"
GRID_H="15"
FOOD_INT="15"
FOOD_N="1"
ALPHA_CHILD="0.5"
FITNESS_MODE="ttd_overall"
PLASTICITY_EVAL="none"
GENOME_GLOB=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --genome-glob) GENOME_GLOB="${2:-}"; shift 2 ;;
    --world) WORLD="${2:-}"; shift 2 ;;
    --episodes) EPISODES="${2:-}"; shift 2 ;;
    --seed-master) SEED_MASTER="${2:-}"; shift 2 ;;
    --max-ticks) MAX_TICKS="${2:-}"; shift 2 ;;
    --grid-w) GRID_W="${2:-}"; shift 2 ;;
    --grid-h) GRID_H="${2:-}"; shift 2 ;;
    --food-spawn-interval) FOOD_INT="${2:-}"; shift 2 ;;
    --food-spawn-n) FOOD_N="${2:-}"; shift 2 ;;
    --alpha-child) ALPHA_CHILD="${2:-}"; shift 2 ;;
    --fitness-mode) FITNESS_MODE="${2:-}"; shift 2 ;;
    --plasticity-eval) PLASTICITY_EVAL="${2:-}"; shift 2 ;;
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
  none) PLAST_ARGS=(--plasticity none) ;;
  p1) PLAST_ARGS=(--plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick) ;;
  p2) PLAST_ARGS=(--plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick) ;;
  *) echo "[error] --plasticity-eval must be none|p1|p2, got: $PLASTICITY_EVAL" >&2; exit 2 ;;
esac

ROOT="$(cd "$(dirname "$0")" && pwd)"
OUT_BASE="$ROOT/FinalGenome_TTD_eval/$WORLD"
mkdir -p "$OUT_BASE"

genomes=( $GENOME_GLOB )
if [[ ${#genomes[@]} -eq 0 ]]; then
  echo "[error] glob matched 0 files: $GENOME_GLOB" >&2
  exit 2
fi

echo "[info] world=$WORLD (thr=$THR) episodes=$EPISODES seed_master=$SEED_MASTER plasticity_eval=$PLASTICITY_EVAL"
echo "[info] matched genomes: ${#genomes[@]}"

for g in "${genomes[@]}"; do
  if [[ ! -f "$g" ]]; then
    echo "[warn] missing file: $g" >&2
    continue
  fi
  tag="$(basename "$(dirname "$g")")"
  out_dir="$OUT_BASE/$tag"
  echo "[run] $tag  <-  $g"

  python3 run_evolve_lineage.py \
    --generations 0 --episodes "$EPISODES" --max-ticks "$MAX_TICKS" \
    --grid-w "$GRID_W" --grid-h "$GRID_H" --threats "$THR" \
    --food-spawn-interval "$FOOD_INT" --food-spawn-n "$FOOD_N" \
    --fitness-mode "$FITNESS_MODE" --alpha-child "$ALPHA_CHILD" \
    --seed-master "$SEED_MASTER" --init-seed "$SEED_MASTER" \
    --init-mode from_json --init-from "$g" \
    "${PLAST_ARGS[@]}" \
    --output-dir "$out_dir"
done

echo "[ok] wrote outputs under: $OUT_BASE"

