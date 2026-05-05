# Experiment plan: plasticity vs evolution

Run from the **project root** in PowerShell.

**Output root:** all experiment batches, genomes, and comparison plots use **`result_experiment\`** (not `result_experiment\`). If you already have runs under `result_experiment\`, copy or move those folders into `result_experiment\`, or change `--genome` / `--output-dir` paths to match your disk.

---

## S1. (One-time) Baseline-zero genome JSON

Used by every “plastic from scratch” experiment.

```powershell
python -c "import os, json; from run_evolve_lineage import baseline_zero_motivation_genome as g; os.makedirs('result_experiment', exist_ok=True); json.dump(g(), open('result_experiment/baseline_zero_genome.json','w'), indent=2)"
```

**Output:** `result_experiment/baseline_zero_genome.json` (all weights = 0.5).

---

## S2. (One-time, optional) CLI plasticity flags on `run_evolve_lineage.py`

If you want **E3** without editing code each time, add flags (`--plasticity-rule`, `--plasticity-update-mode`, `--plasticity-deficit-signal`) and pass them through `_default_cfg()` → `run_episode()`. Otherwise edit `run_evolve_lineage.py` around the `World(...)` call in `run_episode` manually for E3.

---

# E0. Diagnostic — does plasticity actually move weights?

**Goal:** Check whether `u_plastic` moves and whether the direction is consistent across seeds.

```powershell
# E0a — plastic on top of evolved genome
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\evolve_overall_ttd_0_7_threat_10k\final_genome.json `
  --replicates 32 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\E0a_plastic_on_evolved

# E0b — plastic on top of baseline-zero
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\baseline_zero_genome.json `
  --replicates 32 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\E0b_plastic_on_baseline
```

Aggregate plots (mean ± std across seeds; includes motivation / survival):

```powershell
python plot_logged_run_aggregate.py --genome result_experiment\evolve_overall_ttd_0_7_threat_10k\final_genome.json `
  --n-seeds 32 --max-ticks 1000 --out result_experiment\figures\E0a --label "E0a plastic on evolved"

python plot_logged_run_aggregate.py --genome result_experiment\baseline_zero_genome.json `
  --n-seeds 32 --max-ticks 1000 --out result_experiment\figures\E0b --label "E0b plastic on baseline"
```

**Weight drift (E0a / E0b):** `baseline_passive_lower_bound.py` writes `passive_lower_bound_timeseries_*.csv` when timeseries logging is on (default: `--log-timeseries`). Plot aggregate drift and per-term Δ(plastic−fixed) vs tick:

```powershell
python plot_weight_drift.py `
  --input-dir result_experiment\E0a_plastic_on_evolved `
  --out result_experiment\figures\E0a_weight_drift.png
```

This produces `E0a_weight_drift.png` (mean ± std of `u_drift` and `w_drift` across replicates) and `E0a_weight_drift_per_term.png` for any `u_*_fixed` / `u_*_plastic` pairs in those CSVs.

For **full motivation weight terms** (all `m0_u_fixed_*` / `m0_u_plastic_*` columns), use **`headless_rollout_log.py`** with the **same plasticity flags as E0a**, then `plot_weight_drift.py --runlog-glob`.

**Multiple seeds (recommended):** one command writes `run_<seed>.csv` for each seed (`--seed` = first seed, then `+1`, …):

```powershell
python headless_rollout_log.py `
  --genome result_experiment\evolve_overall_ttd_0_7_threat_10k\final_genome.json `
  --seed 10000 --n-seeds 32 --out-dir result_experiment\E0a_runlogs --max-ticks 1000 `
  --grid-w 10 --grid-h 10 `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 --threats 1

python plot_weight_drift.py --runlog-glob "result_experiment\E0a_runlogs\*.csv" `
  --out result_experiment\figures\E0a_runlog_drift.png
```

Single seed: omit `--n-seeds` / `--out-dir` and use `--csv my_run.csv` (default `--n-seeds 1`).

**Headless vs `baseline_passive_lower_bound.py`**

| | **headless_rollout_log.py** | **baseline_passive_lower_bound.py** |
|---|-----------------------------|-------------------------------------|
| **Output** | One wide CSV per run: `tick`, `m0_u_fixed_*`, `m0_u_plastic_*`, motivations, mother/child states (RunLogger schema) | Per-replicate `passive_lower_bound_timeseries_*.csv` (lighter columns) + batch `survival_experiment_replicates.csv` |
| **Best for** | `plot_logged_run.py`, `plot_weight_drift.py --runlog-glob`, per-tick weight trajectories | Many replicates, survival stats, `plot_conditions.py` boxplots |
| **Simulation** | Same `World.step()` pipeline when plasticity flags match | Same pipeline |
| **Grid default** | 30×30 in script (override `--grid-w` / `--grid-h` **10** to match E0a if needed) | Defaults in that script (check `--grid-w` there) |

If E0a used grid 10×10, add to the headless command: `--grid-w 10 --grid-h 10` so runs are comparable.

**Interpret:**

- From per-seed logs / CSVs: **`Δu = u_plastic_end − u_fixed`** per term (mean and std across seeds).
- `|Δu| < 0.05` everywhere → plasticity barely learning (raise `learning_rate` in `core/agents.py` or `learning_rate_max` for adaptive rules).
- Δu large + **same sign across seeds** → real learning signal.
- Δu large + **random sign** → noise; fix credit assignment (E1: `segment_capped` + `local`).

---

# E1. Plastic-only — what within-lifetime learning can do

**Goal:** Compare plastic configurations vs fixed weights, all from baseline-zero genome.

```powershell
# E1a — fixed weights (control)
python baseline_passive_lower_bound.py --mode active --plasticity none `
  --genome result_experiment\baseline_zero_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E1a_fixed

# E1b — segment_capped + local (recommended)
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\baseline_zero_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E1b_plastic_local_seg

# E1c — global deficit, per_tick
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome --deficit-signal global --learn-w off `
  --update-mode per_tick `
  --genome result_experiment\baseline_zero_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E1c_plastic_global_pertick

# E1d — local deficit, per_tick
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome --deficit-signal local --learn-w off `
  --update-mode per_tick `
  --genome result_experiment\baseline_zero_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E1d_plastic_local_pertick
```

Cross-condition boxplots / KM-style comparisons:

```powershell
python plot_conditions.py --root result_experiment
```

**Interpret:**

- **E1b vs E1a** = honest plasticity gain from mediocre innate prior.
- **E1c vs E1d** = global vs local (same update timing).
- **E1b vs E1c** = credit assignment (`segment_capped` vs `per_tick`).
- **Expected ranking (often):** E1b ≥ E1d ≥ E1c ≥ E1a. If E1b < E1a, plasticity may be harmful → revisit E0 (`Δu`).

---

# E2. Evolution-only — innate prior

**Goal:** Strong `u_fixed` from Darwinian search; compare to E1.

```powershell
python run_evolve_lineage.py `
  --generations 1000 --episodes 32 --max-ticks 1000 `
  --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 `
  --seed-master 42 `
  --output-dir result_experiment\E2_evolve_only_seed42

python run_evolve_lineage.py `
  --generations 1000 --episodes 32 --max-ticks 1000 `
  --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 `
  --seed-master 7 `
  --output-dir result_experiment\E2_evolve_only_seed7

python run_evolve_lineage.py `
  --generations 1000 --episodes 32 --max-ticks 1000 `
  --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 `
  --seed-master 99 `
  --output-dir result_experiment\E2_evolve_only_seed99
```

Optional: aggregate rollouts for one lineage:

```powershell
python plot_logged_run_aggregate.py `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --n-seeds 64 --max-ticks 1000 --out result_experiment\figures\E2_seed42 --label "E2 evolved seed42"
```

**Interpret:**

- `lineage_plot.png`: fitness plateau; generations to convergence.
- **3 lineages** → reproducibility of evolved TTD / genome.
- Best E2 TTD vs E1 → **innate vs plastic-from-scratch** core comparison.

---

# E3. Baldwin — plasticity during evolution

**Goal:** Does lifetime plasticity help or hurt the evolutionary search?

`run_evolve_lineage.py` now accepts the same plasticity knobs as `baseline_passive_lower_bound.py`. Default remains **genes-only** (`--plasticity none`).

**Recommended E3 run** (match E1b-style credit assignment; use **≥32 episodes** to denoise fitness):

```powershell
python run_evolve_lineage.py `
  --generations 500 --episodes 32 --max-ticks 1000 `
  --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 `
  --seed-master 42 `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --output-dir result_experiment\E3_baldwin_seed42
```

**Optional:** lower `learning_rate` in `core/agents.py` (e.g. `0.005`) during E3 so mutations stay visible; revert after the run.

**Genes-only baseline (E2-style)** for the same seed — omit plasticity flags or pass `--plasticity none`:

```powershell
python run_evolve_lineage.py `
  --generations 500 --episodes 32 --max-ticks 1000 `
  --threats 1 --fitness-mode ttd_overall --alpha-child 0.7 `
  --seed-master 42 `
  --output-dir result_experiment\E2_evolve_only_seed42
```

**Interpret (vs E2, same `--seed-master`):**

| Outcome | Meaning |
|--------|---------|
| E3 reaches E2 plateau **faster** | Baldwin — plasticity guides search |
| E3 plateau **higher** than E2 | Synergy |
| E3 plateau **same**, genome more extreme | Genetic assimilation |
| E3 plateau **lower** / stalled | Shielding — plasticity hides genetic signal |

Read **fitness vs generation**, not only final TTD boxplots.

---

# E4. Plastic refinement of evolved prior

**Goal:** Same evolved genome, runtime plasticity on vs off.

```powershell
# E4a — evolved genome, no plasticity
python baseline_passive_lower_bound.py --mode active --plasticity none `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E4a_evolved_only

# E4b — evolved genome + plasticity
python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 64 --max-ticks 1000 --threats 1 `
  --output-dir result_experiment\cond_E4b_evolved_plus_plastic

python plot_conditions.py --root result_experiment
```

**Interpret:**

- **E4b ≥ E4a** → plasticity refines a strong prior (good story).
- **E4b ≈ E4a** → plasticity redundant given good prior.
- **E4b < E4a** → plasticity destabilizes prior; consider **warmup** (skip plastic updates for first N ticks) as E4c after a small code change.

---

# E5. Robustness sweep (optional)

**Goal:** Same conclusions under different horizons / threat / food.

Example: long horizon, no threat:

```powershell
python baseline_passive_lower_bound.py --mode active --plasticity none `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 32 --max-ticks 5000 --threats 0 `
  --output-dir result_experiment\cond_E5_long_easy_evolved

python baseline_passive_lower_bound.py --mode active `
  --plasticity outcome_adaptive --deficit-signal local --learn-w off `
  --update-mode segment_capped --segment-kmax 20 `
  --genome result_experiment\E2_evolve_only_seed42\final_genome.json `
  --replicates 32 --max-ticks 5000 --threats 0 `
  --output-dir result_experiment\cond_E5_long_easy_evolved_plastic
```

**Interpret:** If plasticity only helps when episodes are long / easy, state that explicitly in the thesis.

---

## Suggested run order

| When | What |
|------|------|
| Day 1 AM | S1, E0 |
| Day 1 PM | E1a–E1d (parallel terminals if possible) |
| Day 2 AM | E2 × 3 seeds |
| Day 2 PM | E3 (after code edits) |
| Day 3 AM | E4a, E4b |
| Day 3 PM | E5 if time |

After batches:

```powershell
python plot_conditions.py --root result_experiment
```

---

## Thesis decision tree (after E1, E2, E4)

```
TTD(E1b) > TTD(E1a)?
├─ YES — plasticity-alone helps
│   TTD(E4b) > TTD(E4a)?
│   ├─ YES → Innate + lifetime learning synergistic
│   └─ NO  → Plasticity helps weak prior; redundant with strong prior
└─ NO — plasticity-alone doesn’t help
    TTD(E2) >> TTD(E1a)?
    ├─ YES → Genetic priors dominate in this regime
    └─ NO  → Environment may be too hard; try longer max-ticks or easier food/threat
```

All branches are defensible thesis narratives if tied to the numbers and plots.

---

## Related scripts (quick reference)

| Script | Role |
|--------|------|
| `baseline_passive_lower_bound.py` | Active/passive batches; `--plasticity`, `--deficit-signal`, `--learn-w`, `--update-mode`, `--segment-kmax`, `--genome` |
| `run_evolve_lineage.py` | Evolve motivation genome; `--episodes`, `--generations`, `--max-ticks`, `--threats`, `--fitness-mode`, `--alpha-child` |
| `plot_conditions.py` | `--root result_experiment` → comparison figures |
| `plot_logged_run_aggregate.py` | N seeds, mean±std plots + `survival_agg.png`; `--genome`, `--n-seeds`, `--seed-base`, `--max-ticks`, `--out`, `--label` |
| `headless_rollout_log.py` | Single rollout CSV without Pygame |
| `plot_weight_drift.py` | `u_drift` / `w_drift` and per-term weight change from timeseries or RunLogger CSVs |

---

*Generated for the Thesis maternal-intuition project. All experiment outputs use `result_experiment\`; copy older `test_results\` data there if needed.*
