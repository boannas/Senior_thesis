#!/usr/bin/env bash
# 90 emergence runs (10 seeds × 3 inits × 3 plasticities), sharded: WORKER=0..11
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

WORKER=${WORKER:-0}
if ! [[ "$WORKER" =~ ^[0-9]+$ ]] || (( WORKER < 0 || WORKER > 11 )); then
  echo "[error] Set WORKER=0..11. Got: WORKER=${WORKER:-}" >&2
  exit 2
fi

SEEDS=(62 63 64 65 66 67 68 69 70 71)
INIT_ORDER=(anti_maternal random_uniform pro_maternal)
PLAST_ORDER=(E2 P1 P2)

run_lineage() {
  local plast="$1" init="$2" seed="$3"
  local tag="${plast}_${init}_seed${seed}"
  local out_dir="Emergence_results/normal/${tag}"

  local -a common=(
    python3 run_evolve_lineage.py
    --generations 3000
    --episodes 10
    --max-ticks 1000
    --grid-w 15
    --grid-h 15
    --threats 1
    --food-spawn-interval 15
    --food-spawn-n 1
    --fitness-mode ttd_overall
    --alpha-child 0.5
    --seed-master "${seed}"
    --init-seed "${seed}"
    --init-mode "${init}"
    --init-noise 0.05
    --checkpoint-every 500
    --output-dir "${out_dir}"
  )

  case "$plast" in
    E2) "${common[@]}" --plasticity none ;;
    P1)
      "${common[@]}" \
        --plasticity outcome_adaptive \
        --deficit-signal global \
        --learn-w off \
        --update-mode per_tick
      ;;
    P2)
      "${common[@]}" \
        --plasticity outcome \
        --deficit-signal local \
        --learn-w off \
        --update-mode per_tick
      ;;
    *) echo "[error] bad plast: $plast" >&2; exit 2 ;;
  esac
}

n_run=0
i=0
for plast in "${PLAST_ORDER[@]}"; do
  for init in "${INIT_ORDER[@]}"; do
    for seed in "${SEEDS[@]}"; do
      if (( i % 12 == WORKER )); then
        echo "[worker ${WORKER}] (${n_run}) ${plast} ${init} seed${seed}"
        run_lineage "$plast" "$init" "$seed"
        n_run=$((n_run + 1))
      fi
      i=$((i + 1))
    done
  done
done

echo "[worker ${WORKER}] finished ${n_run} lineage run(s)."