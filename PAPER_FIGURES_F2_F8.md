# Paper figures F2–F8: end-to-end runbook

Companion to the SAB-style figure plan: **F2** survival curves, **F3** single-run trace, **F4** evolved motivation weights, **F5** condition boxplots, **F6** Baldwin (fitness + plasticity drift), **F7** weight-drift / plasticity diagnostics, **F8** robustness (long horizon / easier world).

Run from the **project root** (PowerShell). Outputs assume **`result_experiment\`** exists.

---

## Conventions

- **Baseline batch script:** `baseline_passive_lower_bound.py` defaults to **10×10** grid. **`plot_logged_run_aggregate.py`** and **`headless_rollout_log.py`** default to a **larger** grid unless you pass **`--grid-w 10 --grid-h 10`**. Use the same grid, threats, food, and `max-ticks` across conditions you compare.
- **Genome JSON** is the **nested** dict from `baseline_zero_motivation_genome()` in `run_evolve_lineage.py` (keys `forage`, `care`, `self`, `protect` with nested floats). It is **not** the `motivation_weights_fixed` object used inside the agent at runtime.

---

## Phase 0 — One-time prep

### Baseline-zero genome

```powershell
python -c "import os, json; from run_evolve_lineage import baseline_zero_motivation_genome as g; os.makedirs('result_experiment', exist_ok=True); json.dump(g(), open('result_experiment/baseline_zero_genome.json','w'), indent=2)"
```

### Random `u_fixed` controls (3 JSONs)

Randomize every **leaf** float in the nested genome (same structure as baseline-zero). In PowerShell, pipe a short script to the interpreter:

```powershell
@'
import copy, json, os
import numpy as np
from run_evolve_lineage import baseline_zero_motivation_genome

def randomize_leaves(d, rng):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out[k] = randomize_leaves(v, rng)
        else:
            out[k] = float(rng.uniform(0.0, 1.0))
    return out

os.makedirs("result_experiment/random_u", exist_ok=True)
for seed in (13, 17, 23):
    rng = np.random.default_rng(seed)
    G = randomize_leaves(copy.deepcopy(baseline_zero_motivation_genome()), rng)
    path = f"result_experiment/random_u/random_u_seed{seed}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(G, f, indent=2)
    print("Wrote", path)
'@ | python -
```

On bash/zsh you can save the same code to `make_random_u.py` and run `python make_random_u.py`.

---

## Phase 1 — Experiments

### 1a. Evolution lineages (E2 genes-only and E3 Baldwin), matched seeds

Use the **same** `--seed-master` for E2 vs E3 pairs so F6 is a fair comparison.

**E2 — genes-only (no plasticity during evolution):**

```powershell
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 42 --output-dir result_experiment\E2_evolve_only_seed42
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 7  --output-dir result_experiment\E2_evolve_only_seed7
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 99 --output-dir result_experiment\E2_evolve_only_seed99
```

**E3 — Baldwin (plasticity during evolution):**

```powershell
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 42 --plasticity outcome_adaptive --deficit-signal local --learn-w off --update-mode segment_capped --segment-kmax 20 --output-dir result_experiment\E3_baldwin_seed42
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 7  --plasticity outcome_adaptive --deficit-signal local --learn-w off --update-mode segment_capped --segment-kmax 20 --output-dir result_experiment\E3_baldwin_seed7
python run_evolve_lineage.py --generations 1000 --episodes 32 --max-ticks 1000 --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 --seed-master 99 --plasticity outcome_adaptive --deficit-signal local --learn-w off --update-mode segment_capped --segment-kmax 20 --output-dir result_experiment\E3_baldwin_seed99
```

**Outputs:** each folder gets `final_genome.json`, `lineage_generations.csv`, `lineage_plot.png`, etc.

### 1b. F2 batches — passive vs random vs evolved (for survival / paper panel)

**Passive mother** (no genome; mother does nothing):

```powershell
python baseline_passive_lower_bound.py --mode passive --plasticity none --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_passive_mother
```

**Random innate prior** (active, fixed weights, no plasticity):

```powershell
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\random_u\random_u_seed13.json --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_random_u_seed13
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\random_u\random_u_seed17.json --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_random_u_seed17
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\random_u\random_u_seed23.json --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_random_u_seed23
```

**Evolved innate prior** (active, fixed weights, no plasticity):

```powershell
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\E2_evolve_only_seed42\final_genome.json --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_evolved_seed42
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\E2_evolve_only_seed7\final_genome.json  --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_evolved_seed7
python baseline_passive_lower_bound.py --mode active --plasticity none --genome result_experiment\E2_evolve_only_seed99\final_genome.json --replicates 64 --max-ticks 1000 --threats 1 --output-dir result_experiment\F2_evolved_seed99
```

Each batch writes `survival_experiment_replicates.csv` and (if timeseries logging is on) per-replicate `passive_lower_bound_timeseries_*.csv`.

### 1c. F5 — E1 / E4 condition batches

Follow **`EXPERIMENT_PLAN.md`** sections **E1** (E1a–E1d) and **E4** (E4a–E4b), pointing `--output-dir` under `result_experiment\`. Then run **`plot_conditions.py`** (see F5 below).

### 1d. F3 — one headless trace CSV

Match grid to baseline (**10×10**):

```powershell
python headless_rollout_log.py --genome result_experiment\E2_evolve_only_seed42\final_genome.json --seed 10042 --n-seeds 1 --csv result_experiment\F3_trace_seed42.csv --max-ticks 1000 --grid-w 10 --grid-h 10 --threats 1
```

### 1e. F7 — plasticity actually moves weights (E0-style replicates)

**Purpose:** Publication panel showing **mean ± std** of `u_drift` (and optional per-term `Δu`) across seeds, on an evolved genome with plasticity on.

**Option A — baseline batches + `plot_weight_drift.py --input-dir`:** uses default timeseries logging (`--log-timeseries` is on by default in `baseline_passive_lower_bound.py` unless you pass `--no-log-timeseries`).

```powershell
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 32 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\F7_plastic_on_evolved_seed42
```

**Option B — headless RunLogger CSVs (full `m0_u_fixed_*` / `m0_u_plastic_*` columns):** match **10×10** grid to baseline.

```powershell
python headless_rollout_log.py `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --seed 30000 --n-seeds 32 --out-dir result_experiment\F7_runlogs_seed42 --max-ticks 1000 `
  --grid-w 10 --grid-h 10 --threats 1 `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20
```

Repeat with other evolved lineages (`E2_evolve_only_seed7`, etc.) if you want **mean across lineages** in the figure (combine CSVs or run `plot_weight_drift` per folder and assemble in Illustrator / a small script).

### 1f. F8 — robustness sweep (E5-style)

**Purpose:** Show that main conclusions (e.g. E4a vs E4b, or evolved vs plastic) still hold under **different horizon** and/or **easier ecology** (example: long episodes, no threats).

Pick one reference evolved genome (here: seed 42). Adjust paths if you use another lineage.

```powershell
python baseline_passive_lower_bound.py --mode active --plasticity none `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 32 --max-ticks 5000 --threats 0 `
  --output-dir result_experiment\F8_long_easy_evolved_fixed

python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 32 --max-ticks 5000 --threats 0 `
  --output-dir result_experiment\F8_long_easy_evolved_plastic
```

Add more **F8_** folders (e.g. different `max-ticks`, `threats`, food spawn) following **`EXPERIMENT_PLAN.md`** § E5. Then include them in **`plot_conditions.py --root`** (or build a dedicated comparison table / figure from `survival_experiment_replicates.csv`).

---

## Phase 2 — Build each figure

### F2 — Child (or family) survival: passive vs random vs evolved

**Path A — aggregate from genome (random / evolved only):**  
`plot_logged_run_aggregate.py` requires **`--genome`** and runs **N fresh rollouts** (it does **not** read an existing batch folder). Pass **`--grid-w 10 --grid-h 10`** and the same **`--max-ticks`**, **`--threats`**, and food flags as your baseline batches.

Example (one evolved lineage; repeat for other genomes / seeds):

```powershell
python plot_logged_run_aggregate.py --genome result_experiment\E2_evolve_only_seed42\final_genome.json --n-seeds 64 --seed-base 20000 --max-ticks 1000 --threats 1 --grid-w 10 --grid-h 10 --out result_experiment\figures\F2_evolved_seed42_agg --label "F2 evolved seed42"
```

Use **`survival_agg.png`** inside that output directory as the survival panel for that condition.

**Passive mother:** there is no genome for `plot_logged_run_aggregate.py`. Use the **`F2_passive_mother`** batch: either build the survival curve from **`passive_lower_bound_timeseries_*.csv`** (fraction alive per tick), or export Kaplan–Meier-style curves from **`survival_experiment_replicates.csv`** (`child_death_tick` per replicate). For a **single** publication figure overlaying all three conditions, you will likely want a **small custom script** that plots fraction-alive vs tick for passive + two genome-driven conditions on the same axes.

### F3 — Behavior trace (motivations / states)

```powershell
python plot_logged_run.py result_experiment\F3_trace_seed42.csv --out result_experiment\figures\F3_trace_seed42
```

### F4 — Evolved `u_fixed` vs baseline / random (bar or heatmap)

Not automated in-repo yet. Plan: load JSON genomes, flatten nested leaves to a consistent key order, then plot **mean ± across seeds** for evolved vs single baseline vs mean of random JSONs. A dedicated script (e.g. `plot_genome_weights.py`) can take paths listed in Phase 0 and 1a outputs.

### F5 — Boxplots across plastic / evolved conditions

After E1 and E4 batches exist under `result_experiment\`:

```powershell
python plot_conditions.py --root result_experiment
```

Copy or rename the main comparison figure into **`result_experiment\figures\F5_conditions.png`** for the paper.

### F6 — Fitness vs generation: E2 vs E3 + plasticity drift

**Data:** `lineage_generations.csv` in each E2/E3 folder has **`fitness`**, **`mean_u_drift_end`**, **`std_u_drift_end`**, etc.

Not fully automated as a multi-seed overlay in-repo yet. Plan: read the six CSVs, align on **`generation`**, plot **mean ± band across seeds** for **`fitness`** (E2 vs E3), and optionally a **twin y-axis** for **`mean_u_drift_end`** on E3 (and near-zero for E2). A small script (e.g. `plot_baldwin_compare.py`) keeps the figure reproducible.

### F7 — Weight drift: |plastic − fixed| over time and per motivation term

**Claim support:** Plasticity is not decorative — **`u_plastic` diverges from `u_fixed`** in a consistent direction across seeds (see **`EXPERIMENT_PLAN.md`** E0 interpret notes).

**From Option A (baseline timeseries folder):**

```powershell
python plot_weight_drift.py `
  --input-dir result_experiment\F7_plastic_on_evolved_seed42 `
  --out result_experiment\figures\F7_weight_drift_seed42.png
```

This writes **`F7_weight_drift_seed42.png`** (aggregate `u_drift` / `w_drift`) and typically **`F7_weight_drift_seed42_per_term.png`** when per-term columns exist in those CSVs.

**From Option B (headless RunLogger glob):**

```powershell
python plot_weight_drift.py --runlog-glob "result_experiment\F7_runlogs_seed42\*.csv" `
  --out result_experiment\figures\F7_runlog_drift_seed42.png
```

**Paper use:** One panel = mean `u_drift` vs tick ± std across seeds; optional second panel = bar chart of **end-of-run Δu** per term (from `*_per_term` figure or exported CSV). If **|Δu|** is tiny everywhere, fix learning rate / credit assignment before claiming strong plasticity results.

### F8 — Robustness: same comparison under long horizon / no threat (etc.)

After **1f** batches (and any extra F8 conditions) are under `result_experiment\`:

```powershell
python plot_conditions.py --root result_experiment
```

Isolate the **F8_** conditions in the generated comparison, or hand-pick `survival_experiment_replicates.csv` from **`F8_long_easy_evolved_fixed`** vs **`F8_long_easy_evolved_plastic`** and plot child TTD / survival side-by-side (boxplot or ECDF) matching your F5 style.

**Paper use:** Short subsection or appendix: “Under a milder regime (5000 ticks, no threats), we still observe [same ordering as main text / or report a reversal].” Stating **when plasticity only helps** strengthens the paper.

---

## Phase 3 — Suggested schedule

| When | Run | Unblocks |
|------|-----|----------|
| Day 1 | Phase 0 + 1d (F3 trace) | F3 |
| Day 1–2 | 1a E2/E3 × 3 seeds | F4, F6 |
| Day 2 | 1b F2 batches | F2 |
| Day 2–3 | 1c E1/E4 | F5 |
| Day 2–3 | 1e F7 (after at least one E2 genome exists) | F7 |
| Day 3–4 | 1f F8 + `plot_conditions` | F8 |
| Day 3+ | Aggregate plots + optional custom F2/F4/F6 scripts | All panels |

---

## Optional next steps (code)

- **`plot_genome_weights.py`** — F4 from JSON paths.
- **`plot_baldwin_compare.py`** — F6 from multiple `lineage_generations.csv`.
- **`plot_survival_overlay.py`** — F2: passive batch + N `plot_logged_run_aggregate` exports or raw timeseries → one axes.

See also **`EXPERIMENT_PLAN.md`** for E0–E5 definitions and plasticity flags.
