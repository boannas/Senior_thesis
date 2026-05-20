#!/usr/bin/env bash
# Evolve batch: E2 / P1 / P2 × three init modes × seeds 47–51 (45 runs).
#
# Usage:
#   ./run_emergence_normal_seeds47_51.sh
#       Run all jobs one after another.
#
#   WORKER=0 ./run_emergence_normal_seeds47_51.sh   # terminal 1 of 12
#   WORKER=11 ./run_emergence_normal_seeds47_51.sh  # terminal 12 of 12
#       Each WORKER in 0..11 runs a disjoint subset (round-robin). Use 12
#       terminals with WORKER=0 .. WORKER=11 to cover all 45 runs in parallel.

set -euo pipefail

cd "$(dirname "$0")"

TOTAL_WORKERS=12
WORKER="${WORKER:-}"

if [[ -n "$WORKER" ]]; then
  if ! [[ "$WORKER" =~ ^[0-9]+$ ]] || (( WORKER < 0 || WORKER >= TOTAL_WORKERS )); then
    echo "WORKER must be an integer in 0..$((TOTAL_WORKERS - 1)), got: $WORKER" >&2
    exit 1
  fi
fi

run_job() {
  local kind=$1 init=$2 s=$3
  local tag="${kind}_${init}_seed${s}"
  local -a plasticity_args=()
  case "$kind" in
    E2) plasticity_args=( --plasticity none ) ;;
    P1) plasticity_args=(
         --plasticity outcome_adaptive --deficit-signal global
         --learn-w off --update-mode per_tick
       ) ;;
    P2) plasticity_args=(
         --plasticity outcome --deficit-signal local
         --learn-w off --update-mode per_tick
       ) ;;
    *) echo "bad kind: $kind" >&2; exit 1 ;;
  esac

  python3 run_evolve_lineage.py \
      --generations 3000 --episodes 10 --max-ticks 1000 \
      --grid-w 15 --grid-h 15 --threats 1 \
      --food-spawn-interval 15 --food-spawn-n 1 \
      --fitness-mode ttd_overall --alpha-child 0.5 \
      --seed-master "$s" --init-seed "$s" \
      --init-mode "$init" --init-noise 0.05 \
      "${plasticity_args[@]}" \
      --checkpoint-every 500 \
      --output-dir "Emergence_results/normal/$tag"
}

idx=0
for kind in E2 P1 P2; do
  for init in anti_maternal random_uniform pro_maternal; do
    for s in 47 48 49 50 51; do
      if [[ -z "$WORKER" ]] || (( idx % TOTAL_WORKERS == WORKER )); then
        run_job "$kind" "$init" "$s"
      fi
      ((++idx))
    done
  done
done
