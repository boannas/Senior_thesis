# Run Guide — 12-Terminal Parallel Sweep (Linux)

This document contains the full set of commands to reproduce the SAB-class experiment.
Open **12 Linux terminals**, paste **one batch per terminal**, and they will run independently
with no shared state. Total: **225 runs** (90 emergence + 135 sweep) across 12 batches.

> **Tip:** if you SSH into a remote machine, run everything inside `tmux` so the jobs survive
> disconnects (see [Tmux quickstart](#tmux-quickstart) at the bottom).

---

## Experimental design at a glance

| Block | Purpose | Cells | Seeds / cell | Runs |
|---|---|---:|---:|---:|
| **Emergence** (Batches 1–3) | "Maternal phenotype emerges from any initial state" | 3 inits × 3 plasticities = 9 | 10 | 90 |
| **Plasticity sweep** (Batches 4–12) | "Plasticity is differentially useful across (world, alpha)" | 3 worlds × 3 alphas × 3 plasticities = 27 | 5 | 135 |
| **Total** | | 36 | | **225** |

Constants used everywhere:

| Parameter | Value | Notes |
|---|---|---|
| `--generations` | 5000 | full evolutionary run |
| `--episodes` | 10 | seeds per fitness evaluation |
| `--max-ticks` | 1000 | per episode |
| `--grid-w / --grid-h` | 15 / 15 | representative grid |
| `--food-spawn-interval / -n` | 15 / 1 | constant resource pressure |
| `--fitness-mode` | `ttd_overall` | child + mother time-to-death |

What varies across batches:

- **Plasticity rule**: `none` (E2) vs `outcome_adaptive` + `global` (P1) vs `outcome` + `local` (P2)
- **World difficulty**: `easy` (`--threats 0`) / `normal` (`--threats 1`) / `hard` (`--threats 2`)
- **Initialization** (emergence only): `anti_maternal` / `random_uniform` / `pro_maternal`
- **Fitness weighting** (sweep only): `--alpha-child 0.3 / 0.5 / 0.7`
- **Seeds**: 42–46 (sweep) or 42–51 (emergence). `--seed-master` and `--init-seed` are tied so each replicate has both a different episode RNG and a different init RNG.

Output directory layout (auto-created):

```
Emergence_results/
  normal/
    {plasticity}_{init}_seed{N}/
      lineage_generations.csv
      lineage_plot.png
      init_genome.json
      final_genome.json
      run_config.json
      cli_args.txt
      checkpoints/
        best_gen0500.json
        best_gen1000.json
        ...

Sweep_results/
  {easy|normal|hard}/
    {plasticity}_thr{T}_g15_i15_a{alpha}_seed{N}/
      lineage_generations.csv
      lineage_plot.png
      init_genome.json
      final_genome.json
      run_config.json
      cli_args.txt
```

The 12 batches write to **non-overlapping directories**, so they can run concurrently with no
filename collisions.

---

## Batches 1–3: Emergence (30 runs each)

3 init modes × 10 seeds = 30 runs per batch.

### Batch 1 — Emergence × E2 (no plasticity)

```bash
for init in anti_maternal random_uniform pro_maternal; do
  for s in 42 43 44 45 46 47 48 49 50 51; do
    tag="E2_${init}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats 1 \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child 0.5 \
        --seed-master $s --init-seed $s \
        --init-mode $init --init-noise 0.05 \
        --plasticity none \
        --checkpoint-every 500 \
        --output-dir "Emergence_results/normal/$tag"
  done
done
```

### Batch 2 — Emergence × P1 (outcome_adaptive + global, per_tick)

```bash
for init in anti_maternal random_uniform pro_maternal; do
  for s in 42 43 44 45 46 47 48 49 50 51; do
    tag="P1_${init}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats 1 \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child 0.5 \
        --seed-master $s --init-seed $s \
        --init-mode $init --init-noise 0.05 \
        --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick \
        --checkpoint-every 500 \
        --output-dir "Emergence_results/normal/$tag"
  done
done
```

### Batch 3 — Emergence × P2 (outcome + local, per_tick)

```bash
for init in anti_maternal random_uniform pro_maternal; do
  for s in 42 43 44 45 46 47 48 49 50 51; do
    tag="P2_${init}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats 1 \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child 0.5 \
        --seed-master $s --init-seed $s \
        --init-mode $init --init-noise 0.05 \
        --plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick \
        --checkpoint-every 500 \
        --output-dir "Emergence_results/normal/$tag"
  done
done
```

---

## Batches 4–6: Sweep × E2 (15 runs each)

3 alphas × 5 seeds = 15 runs per batch. World fixed per batch.

### Batch 4 — Sweep × E2 × easy (threats=0)

```bash
WORLD=easy; THR=0
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="E2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity none \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 5 — Sweep × E2 × normal (threats=1)

```bash
WORLD=normal; THR=1
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="E2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity none \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 6 — Sweep × E2 × hard (threats=2)

```bash
WORLD=hard; THR=2
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="E2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity none \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

---

## Batches 7–9: Sweep × P1 (15 runs each)

### Batch 7 — Sweep × P1 × easy

```bash
WORLD=easy; THR=0
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P1_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 8 — Sweep × P1 × normal

```bash
WORLD=normal; THR=1
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P1_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 9 — Sweep × P1 × hard

```bash
WORLD=hard; THR=2
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P1_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

---

## Batches 10–12: Sweep × P2 (15 runs each)

### Batch 10 — Sweep × P2 × easy

```bash
WORLD=easy; THR=0
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 11 — Sweep × P2 × normal

```bash
WORLD=normal; THR=1
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

### Batch 12 — Sweep × P2 × hard

```bash
WORLD=hard; THR=2
for a in 0.3 0.5 0.7; do
  atag=${a/./p}
  for s in 42 43 44 45 46; do
    tag="P2_thr${THR}_g15_i15_a${atag}_seed${s}"
    python3 run_evolve_lineage.py \
        --generations 5000 --episodes 10 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $THR \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child $a \
        --seed-master $s --init-seed $s \
        --init-mode baseline_zero \
        --plasticity outcome --deficit-signal local --learn-w off --update-mode per_tick \
        --output-dir "Sweep_results/$WORLD/$tag"
  done
done
```

---

## Before launching: estimate runtime

Run *one* short command to time it, then extrapolate:

```bash
time python3 run_evolve_lineage.py \
    --generations 100 --episodes 10 --max-ticks 1000 \
    --grid-w 15 --grid-h 15 --threats 1 \
    --food-spawn-interval 15 --food-spawn-n 1 \
    --fitness-mode ttd_overall --alpha-child 0.5 \
    --seed-master 99 --init-seed 99 \
    --init-mode baseline_zero \
    --plasticity outcome_adaptive --deficit-signal global --learn-w off --update-mode per_tick \
    --output-dir /tmp/rt_test
```

Multiply the wall-clock by **50** (5000 generations / 100) to estimate one full run.
Multiply that by the batch size (30 for emergence batches, 15 for sweep batches) to estimate
total wall-clock per batch.

---

## Monitoring progress

```bash
# Row count = current generation
wc -l Emergence_results/normal/P1_anti_maternal_seed42/lineage_generations.csv

# Latest 3 rows (generation, fitness, accepted):
tail -3 Emergence_results/normal/P1_anti_maternal_seed42/lineage_generations.csv | cut -d, -f1,2,3

# Quick check: how many runs in a batch have produced a final genome yet?
ls Emergence_results/normal/*/final_genome.json 2>/dev/null | wc -l

# Disk usage so far:
du -sh Emergence_results Sweep_results 2>/dev/null
```

---

## After all 12 batches finish: aggregation and plots

### 1. Sanity check: every run completed

```bash
# Should print "30 30 30 15 15 15 15 15 15 15 15 15"
for d in Emergence_results/normal Sweep_results/easy Sweep_results/normal Sweep_results/hard; do
  for plast in E2 P1 P2; do
    n=$(ls $d/${plast}_*/final_genome.json 2>/dev/null | wc -l)
    echo "$d $plast -> $n"
  done
done
```

If any cell has fewer files than expected, find the broken run via:

```bash
for d in Emergence_results/normal/*/; do
  if [ ! -f "$d/final_genome.json" ]; then
    echo "INCOMPLETE: $d"
    tail -n 3 "$d/lineage_generations.csv" 2>/dev/null
  fi
done
```

…then re-launch only those commands.

### 2. Build the tidy summary CSV

```bash
python3 summarize_lineage_runs.py
```

(Adjust the input directories inside that script if needed; it should walk
`Emergence_results/` and `Sweep_results/`.)

### 3. Headline figure: Δ-fitness heatmap

```bash
python3 plot_baldwin_heatmap.py \
    --summary Sweep_results/lineage_summary.csv \
    --asymp-window 1000 \
    --out-dir Sweep_results/figures/heatmaps
```

This produces:
- `heatmap_fitness_E2_P1_delta.png` — the headline plot (Δ = P1 − E2)
- `heatmap_fitness_P2_delta.png`
- `heatmap_u_drift.png`
- `heatmap_data.csv`, `per_run_asymp.csv` — the raw numbers

---

## Claim C (optional): the non-learner test

This is the genetic-assimilation test. After Batches 1–3 finish, we use saved checkpoints
from any P1 emergence run and evaluate them with **plasticity disabled**:

```bash
SEED=42
RUN_DIR="Emergence_results/normal/P1_anti_maternal_seed${SEED}"

for g in 1000 2500 5000; do
  for w in easy normal hard; do
    case $w in
      easy)   thr=0 ;;
      normal) thr=1 ;;
      hard)   thr=2 ;;
    esac
    ck="$RUN_DIR/checkpoints/best_gen$(printf '%04d' $g).json"
    python3 run_evolve_lineage.py \
        --generations 0 --episodes 30 --max-ticks 1000 \
        --grid-w 15 --grid-h 15 --threats $thr \
        --food-spawn-interval 15 --food-spawn-n 1 \
        --fitness-mode ttd_overall --alpha-child 0.5 \
        --seed-master 100 \
        --init-mode from_json --init-from "$ck" \
        --plasticity none \
        --output-dir "NonLearner_test/$w/from_g${g}"
  done
done
```

Reads in CSV after:

```bash
for d in NonLearner_test/*/from_g*/; do
  echo -n "$d "
  python3 -c "import csv; r=list(csv.DictReader(open('$d/lineage_generations.csv'))); print('fitness=',r[0]['fitness'])"
done
```

If late-generation genomes still perform well *without plasticity* under hard environments,
that is direct evidence of genetic assimilation.

---

## Tmux quickstart

If you SSH into a remote machine, wrap the 12 terminals in tmux so the runs survive
disconnects, screen lock, and laptop close:

```bash
tmux new -s evolve              # start a new tmux session
# Inside tmux:
#   Ctrl+b then "    -> split horizontally
#   Ctrl+b then %    -> split vertically
#   Ctrl+b then arrows -> move between panes
#   Ctrl+b then z     -> zoom into / out of current pane
#   Ctrl+b then d     -> DETACH (runs keep going)

# Reattach later:
tmux attach -t evolve

# List all sessions:
tmux ls

# Kill the session (after you're done):
tmux kill-session -t evolve
```

A nice 12-pane layout: split horizontally 3 times, then split each row vertically into 4.

---

## Reproducibility notes

Each run writes the following to its `--output-dir`, so any run can be replayed exactly later:

- `run_config.json` — full configuration, CLI command line, Python version, git short hash, timestamp
- `cli_args.txt` — the literal command line that launched the run
- `init_genome.json` — the gen-0 genome (after `--init-noise` is applied)
- `final_genome.json` — best genome at the end of evolution
- `checkpoints/best_gen{N:04d}.json` — best-so-far genomes at intervals (Batches 1–3 only,
  via `--checkpoint-every 500`)

If you ever need to re-run a single command, the easiest way is:

```bash
# Re-run with the exact original command line:
$(cat <output_dir>/cli_args.txt)
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `--init-mode from_json requires --init-from PATH` | typo in the command | check the path exists |
| `lineage_generations.csv` missing the final row | run was killed mid-generation | re-launch only that command |
| Two runs writing to the same directory | wrong `--output-dir` template | check `tag` variable in the loop |
| "Permission denied" creating directory | running outside repo / wrong cwd | `cd` to repo root first |
| `python3: command not found` | use `python` instead | adjust the loops |

---

## TL;DR

1. Launch 12 terminals (or 12 tmux panes).
2. Paste one batch into each.
3. Wait. Use the monitoring snippets to track progress.
4. Run `summarize_lineage_runs.py` then `plot_baldwin_heatmap.py`.
5. Optionally run the non-learner test for the assimilation claim.
