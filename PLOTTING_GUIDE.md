# Plotting guide — all `plot_*.py` scripts

Reference for every plotting script in this repo: **what it plots**, **required inputs**, and **CLI arguments**.

**Common input layout (final-genome rollouts):**

```
FinalGenomeRollouts_<world>/
  E2_anti_maternal_seed42/run_20000.csv
  P1_random_uniform_seed44/run_20001.csv
  ...
```

Each `run_*.csv` comes from `headless_rollout_log.py`. Folder names must match  
`{E2|P1|P2}_{anti_maternal|random_uniform|pro_maternal}_seed{N}`.

**Alive ticks convention (most scripts):** both `m0_energy` and `c0_hunger` finite.  
**Child death in log:** first tick where `c0_hunger` is non-finite.

---

## Quick pipeline (thesis rollouts)

| Step | Script | Purpose |
|------|--------|---------|
| 1 | `plot_runlog_ttd_across_seeds.py` | Per-genome `ttd_across_seeds.csv` |
| 2 | `plot_runlog_conditional_behavior.py --extended` | `conditional_metrics_single.csv` |
| 3 | Downstream plots | Stacks, deltas, signatures, terminal risk, … |

---

## A. Rollout logs — behavior & conditionals

### `plot_runlog_conditional_behavior.py`

**Plots / outputs:** Conditional maternal metrics from `run_*.csv`; boxplot grids by **init × plasticity** (one PNG per metric). Optional multi-stage trajectories.

**Core metrics:** `P_maternal_given_hungry`, `P_forage_given_hungry`, `P_protect_given_fear`, mean closeness/injury/fear, `child_ttd_norm`, `mother_ttd_norm`.

**With `--extended`:** All `P_sel_{Forage,Care,Self,Protect}_given_{child_hungry,child_cold,child_injured,mother_fear_hi,mother_energy_lo,mother_stress_hi}` plus physiology means.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | — | Single rollout root (`<plast>_<init>_seedN/run_*.csv`) |
| `--stage-root` | — | Repeatable `label=path` for initial/transient/emergent |
| `--out-dir` | auto | Output directory |
| `--extended` | off | Extended masks + many extra PNGs in `extended/` subdir |
| `--extended-subdir` | `extended` | Subfolder for extended-only figures |
| `--hunger-tau` | 50 | Child hungry: `c0_hunger > τ` |
| `--fear-tau` | 10 | Mother afraid: `m0_fear_threat > τ` |
| `--warmth-cold-tau` | 47 | Child cold: `c0_warmth < τ` (50 = comfortable) |
| `--injury-tau` | 1 | Child injured: `c0_injury > τ` |
| `--energy-low-tau` | 50 | Mother low energy: `m0_energy < τ` |
| `--stress-high-tau` | 40 | Mother high stress: `m0_stress > τ` |
| `--title-prefix` | `""` | Figure title prefix |

**Outputs:** `conditional_metrics_single.csv`, `cond_<metric>_single.png` (and per-stage variants).

```bash
python3 plot_runlog_conditional_behavior.py \
  --root FinalGenomeRollouts_normal_seen --extended
```

---

### `plot_conditional_motivation_stacks.py`

**Plots:** All 6 state masks × 4 motivations — **100% stacked bars** and **heatmap** (plasticity × init), one world.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | — | Recompute from run logs |
| `--from-csv` | — | Use existing extended `conditional_metrics_single.csv` |
| `--out-dir` | auto | Output directory |
| `--title` | `""` | Title suffix |
| `--hunger-tau` … `--stress-high-tau` | (same as above) | Only if `--root` |

**Outputs:** `motivation_stacks_by_mask.png`, `motivation_heatmap_by_mask.png`, `motivation_by_mask_summary.csv`

---

### `plot_conditional_motivation_world_delta.py`

**Plots:** **ΔP(sel\|mask) = unseen − base**, genome-matched; median Δ heatmap (3×3: plasticity × init).

| Argument | Default | Description |
|----------|---------|-------------|
| `--base-root` / `--base-csv` | — | Training / reference world |
| `--unseen` | — | Repeatable `tag=root` or `tag=csv` |
| `--out-dir` | required | Output directory |
| `--vmax` | auto | Cap diverging colormap |

**Outputs:** `delta_median_heatmap__<tag>.png`, `delta_per_genome__<tag>.csv`, `delta_median_table__<tag>.csv`

---

### `plot_init_sensitivity_signature.py`

**Plots:** (1) **Sensitivity profiles** — lines of P(sel\|mask) over 6 masks, per init/world; (2) **anti−pro contrast heatmap**; (3) **2D behavioral scatter** (Care vs Forage, Care vs Protect|fear).

| Argument | Default | Description |
|----------|---------|-------------|
| `--csv` | — | Repeatable `[label=]path` (comma-merge paths under one label) |
| `--out-dir` | required | Output directory |
| `--title` | `""` | Title suffix |
| `--contrast-a` / `--contrast-b` | anti / pro | Inits for contrast heatmap |

**Outputs:** `sensitivity_profile_{E2,P1,P2}.png`, `contrast_heatmap.png`, `behavioral_scatter_*.png`

---

### `plot_runlog_motivation_fractions_alive.py`

**Plots:** Unconditional fraction of ticks each motivation is selected (alive ticks only); box or violin by init/plasticity.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | Rollout root |
| `--out-dir` | `<root>/figures` | Output |
| `--layout` | `split_plasticity` | `split_init` or `split_plasticity` |
| `--style` | `box` | `box` or `violin` |
| `--title-prefix` | `""` | Title prefix |
| `--metrics` | all four | Comma-separated motivations to plot |

---

### `plot_joint_state_motivation_heatmap.py`

**Plots:** Binned **P(sel=M \| state in bin)** vs continuous state (e.g. hunger); median across genomes; one PNG per state × condition × plasticity.

| Argument | Default | Description |
|----------|---------|-------------|
| `--cond` | required | Repeatable `label=root1,root2` |
| `--out-dir` | required | Output |
| `--state` | (defaults) | Repeatable column, e.g. `c0_hunger`, or `all` |
| `--bins` | 20 | Number of bins |
| `--clip-min` / `--clip-max` | — | Clip state before binning |
| `--condition-order` | input order | Comma-separated labels |
| `--intersection-only` | off | Same genomes in all conditions |

**Outputs:** `heatmap_<state>__<condition>__<plast>.png` + `.csv`

---

### `plot_state_motivation_correlations.py`

**Plots:** Spearman ρ(state, `m0_sel_*`) per run → median ± IQR across genomes vs world condition.

| Argument | Default | Description |
|----------|---------|-------------|
| `--cond` | required | Repeatable `label=root` |
| `--out-dir` | required | Output |
| `--condition-order` | — | X-axis order |
| `--intersection-only` | off | Genome intersection |
| `--state-cols` | default list | Comma-separated state columns |
| `--max-plots` | 60 | Cap on number of PNGs |

**Outputs:** `corr_summary.csv`, `corr_<state>__<mot>.png`, `*_trajectory.png`

---

### `plot_child_terminal_risk_profile.py`

**Plots:** **Pre-death window** child/mother state (last W alive ticks); proxy “death pathway”; distributions by mother init type.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | Rollout root |
| `--out-dir` | auto | `<root>/figures_child_terminal_profile` |
| `--window` | 50 | Last N child-alive ticks before death |
| `--title` | `""` | Title suffix |

**Outputs:** `child_terminal_profile_per_genome.csv`, `terminal_means_by_init.png`, `death_pathway_distribution.png`, `terminal_state_hist_by_mother_type.png`, `terminal_injury_by_seed.png`, `terminal_hunger_by_seed.png`, `spearman_fear_injury_by_init.png`, `death_pathway_counts.csv`

**Proxy pathways:** `hunger_stress`, `injury_stress`, `cold_stress`, `low_all_axes`, `unknown` (from terminal means, not literal cause).

---

## B. Multi-world comparison

### `plot_world_condition_comparison.py`

**Plots:** Compare conditional metrics across worlds — outcomes, responsiveness, states, extended conditions; optional stage trajectories; cross-metric correlations.

| Argument | Default | Description |
|----------|---------|-------------|
| `--cond` | required | Repeatable `label=FinalGenomeRollouts_*` |
| `--out-dir` | required | Output |
| `--ensure-metrics` | off | Run `plot_runlog_conditional_behavior.py` if CSV missing |
| `--force-recompute` | off | Recompute even if CSV exists |
| `--no-extended` | off | Skip `--extended` when computing |
| `--metrics-subdir` | `figures_conditional_extended` | Where CSV lives |
| `--stage-csv` | — | Repeatable `stage=path` for trajectories |
| `--condition-order` | — | Plot x-axis order |
| `--intersection-only` | off | Same genomes in all worlds |

**Outputs:** `compare_outcomes.png`, `compare_responsiveness.png`, `compare_states.png`, `compare_extended_conditions.png`, `condition_summary.csv`, stage trajectory PNGs

---

### `plot_init_signature_food_threat.py`

**Plots:** Init signatures on **food × threat** ecology plane — ecology scatter, PCA/LDA embeddings, signature vs world lines, per-world LDA panels.

| Argument | Default | Description |
|----------|---------|-------------|
| `--out-dir` | required | Output |
| `--cond` | required | Repeatable `label=root` (comma-merge) |
| `--metrics-subdir` | `figures_conditional_extended` | CSV subdir |
| `--condition-order` | — | Label order |
| `--intersection-only` | off | Genome intersection |
| `--metric` | Care\|hungry | Repeatable metric column for ecology plots |
| `--geom` | built-in map | Repeatable `label=food,threats` override |
| `--no-ecology-combined` | off | Skip combined 1×3 ecology figure |
| `--no-signature-world-lines` | off | Skip median±IQR vs world |
| `--no-lda-loadings` | off | Skip LD1 bar chart |
| `--no-lda-per-world` | off | Skip per-world LDA panels |

**Outputs:** `signature_ecology_scatter__*`, `embedding_pca__*`, `embedding_lda__*`, `per_genome_*.csv`, etc.

---

### `plot_behavior_surface_ttd.py`

**Plots:** 3D-style **smooth surface** of `child_ttd_norm` vs two conditional-P axes (triangulation).

| Argument | Default | Description |
|----------|---------|-------------|
| `--cond` | required | Repeatable `label=root` |
| `--x` / `--y` | required | Column names from conditional CSV |
| `--out-dir` | required | Output |
| `--init` | `all` | Filter to one init or all |
| `--metrics-subdir` | `figures_conditional_extended` | CSV location |
| `--grid-n` | 35 | Interpolation grid size |
| `--condition-order` | — | Process order |

---

## C. TTD & transfer

### `plot_runlog_ttd_across_seeds.py`

**Plots:** Mother/child **time-to-death** across rollout seeds in one genome folder.

| Argument | Default | Description |
|----------|---------|-------------|
| `--runlog-dir` **or** `--runlog-glob` | one required | Folder or glob of `run_*.csv` |
| `--day-step` | 100 | Ticks per day (y-axis) |
| `--out` / `-o` | auto | PNG path |
| `--out-csv` | auto | Summary CSV |
| `--title` | `""` | Title prefix |

**Outputs:** `ttd_across_seeds.png`, `ttd_across_seeds.csv`

---

### `plot_finalgenome_rollout_ttd_emergence_style.py`

**Plots:** Paper-style TTD boxplots from many `ttd_across_seeds.csv` under a rollout root.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | Root with per-genome `ttd_across_seeds.csv` |
| `--units` | `norm` | `norm` or `days` |
| `--layout` | `split_init` | `split_init` or `split_plasticity` |
| `--day-step` | 100 | For days mode |
| `--title-prefix` | `""` | Title prefix |

---

### `plot_rollout_ttd_emergence_full_set.py`

**Plots:** Full thesis TTD set — boxplots, paired Δ vs random, forest plots, Bland–Altman.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | Root with `ttd_across_seeds.csv` per genome |
| `--out-dir` | auto | `<root>/figures_thesis_set` |
| `--units` | `norm` | `norm` or `days` |
| `--day-step` | 100 | Day conversion |
| `--equivalence-band` | — | Shade ±band on Δ plots |
| `--bootstrap` | 4000 | Forest CI resamples |
| `--title-prefix` | `""` | Title prefix |

---

### `plot_unseen_world_transfer.py`

**Plots:** **ΔTTD(unseen − base)** per genome; box+jitter and forest plots (child + mother).

| Argument | Default | Description |
|----------|---------|-------------|
| `--base` | required | Base rollout root |
| `--unseen` | required | Repeatable `label=path` |
| `--out-dir` | auto | Output |
| `--units` | `norm` | `norm` or `days` |
| `--day-step` | 100 | Day conversion |
| `--bootstrap` | 4000 | Forest CIs |
| `--title-prefix` | `""` | Title prefix |

**Requires:** `ttd_across_seeds.csv` in each genome folder (from `plot_runlog_ttd_across_seeds.py`).

---

## D. Emergence & evolution

### `plot_emergence_summary.py`

**Plots:** Summarize `Emergence_results/**/lineage_generations.csv` — heatmaps, boxplots, Baldwin-style lineage grids, weight trajectories.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | `Emergence_results/normal` | Tree to walk |
| `--out-dir` | auto | `<root>/emergence_figures` |
| `--asymp-window` | 200 | Trailing gens for asymptotic means |
| `--world` | — | Filter `easy` / `normal` / `hard` |
| `--ma-window` | 50 | MA for grid plots |
| `--max-gen` | — | Clip generations |
| `--no-lineage-grids` | off | Skip grid plots |
| `--weight-grids` | `subset` | `none` / `subset` / `all` |
| `--weight-cols` | — | Comma-separated `u_*` columns |
| `--weight-show-seeds` | off | Faint per-seed lines |
| `--weight-delta` | off | Δ from gen 0 |
| `--distribution-plot` | `boxplot` | `boxplot` or `violin` |

**Outputs:** `emergence_summary.csv`, many `emergence_heatmap_*.png`, `emergence_boxplot_*.png`, `emergence_grid_*.png`

---

### `plot_lineage_generations.py`

**Plots:** Single-run lineage curves from one `lineage_generations.csv`.

| Argument | Default | Description |
|----------|---------|-------------|
| `csv` | `lineage_generations.csv` | Positional path |
| `--out` | — | Output PNG path |
| `--metrics` | — | Which columns to plot |
| `--dpi` | 150 | Resolution |

---

### `plot_checkpoint_weights.py`

**Plots:** Weight heatmaps / trajectories for one emergence run’s `checkpoints/best_gen*.json`.

| Argument | Default | Description |
|----------|---------|-------------|
| `run_dir` | positional | Run folder with `checkpoints/` |
| `--out-dir` | auto | Output |
| `--dpi` | 150 | Resolution |
| `--heatmap-only` | off | Only heatmap |
| `--no-csv` | off | Skip CSV export |

---

### `plot_emergence_checkpoint_weights_mean_heatmap.py`

**Plots:** **Mean** weight across many runs at each checkpoint generation (aggregate heatmap).

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | e.g. `Emergence_results/normal` |
| `--out-dir` | auto | Output |
| `--out-png` | `emergence_checkpoint_weights_mean_heatmap.png` | Filename |
| `--align` | `nanmean` | `nanmean` or `intersection` |
| `--plasticity` / `--init` | — | Filter one type |
| `--vmin` / `--vmax` | 0 / 1 | Color scale |
| `--title` | `""` | Title override |

---

### `plot_lineage_weights_overlay.py`

**Plots:** Overlay weight trajectories for multiple seeds/inits/plasticities from lineage CSVs.

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | required | Parent of run subdirs |
| `--seed` / `--seeds` | — | Which seeds |
| `--inits` | all three | Init types |
| `--plasticities` | E2,P1,P2 | Plasticities |
| `--ma-window` | 50 | Smoothing |
| `--max-gen` | — | Clip |
| `--out-dir` | auto | Output |
| `--no-l2` | off | Skip L2 plot |

---

### `plot_weight_drift.py`

**Plots:** Plastic vs fixed weight drift from passive baseline timeseries or runlog `m0_u_*` columns.

| Argument | Default | Description |
|----------|---------|-------------|
| `--input-dir` | — | Folder with passive timeseries CSVs |
| `--runlog-glob` | — | Optional run log glob |
| `--out` | required | Output PNG path |

---

## E. Baldwin / sweep experiments

### `plot_baldwin_heatmap.py`

**Plots:** Heatmaps over world × α from `lineage_summary.csv` (sweep results).

| Argument | Default | Description |
|----------|---------|-------------|
| `--summary` | `Baldwin_Experiment/lineage_summary.csv` | Input CSV |
| `--asymp-window` | 1000 | Asymptotic window |
| `--out-dir` | `Baldwin_Experiment/figures/heatmaps` | Output |
| `--cmap-fitness` / `--cmap-drift` / `--cmap-delta` | colormaps | Color maps |

---

### `plot_baldwin_grid.py` / `plot_baldwin_full_grid.py`

**Plots:** Multi-panel MA trajectories (fitness, TTD, drift, etc.) across sweep cells.

Key args: `--root`, `--metric`, `--ma-window`, `--band`, `--share-y`, `--ymin`/`--ymax`, `--dpi`  
(See `--help` on each script for full list.)

---

### `plot_threat_schedule.py`

**Plots:** Fitness / drift vs generation with **threat-schedule** regimes (sweep with time-varying threat).

Key args: `--root`, `--csv`, `--metric`, `--ma-window`, `--worlds`, `--alphas`, `--plasticities`, `--overlay`, `--dpi`

---

## F. Single-run & baseline visualization

### `plot_logged_run.py`

**Plots:** Time series from **one** `run_*.csv` — mother/child states, motivations, weights.

| Argument | Default | Description |
|----------|---------|-------------|
| `csv_path` | `run_log.csv` | Input CSV |
| `--out` | — | Output directory for PNGs |
| `--no-show` | — | Non-interactive |
| (see `--help`) | | Subset of panels |

---

### `plot_logged_run_aggregate.py`

**Plots:** Mean ± variance across **many seeds** for one genome (runs rollouts internally).

| Argument | Default | Description |
|----------|---------|-------------|
| `--genome` | required | JSON genome |
| `--n-seeds` | 16 | Number of rollouts |
| `--seed-base` | 10000 | First seed |
| `--max-ticks` | 10000 | Horizon |
| `--out` | `figures_agg` | Output dir |
| Grid/food/threat/plasticity flags | (defaults) | Same as `headless_rollout_log.py` |

---

### `plot_conditions.py`

**Plots:** Survival and state trajectories from **`baseline_passive_lower_bound.py`** outputs (`survival_experiment_replicates.csv`, timeseries).

| Argument | Default | Description |
|----------|---------|-------------|
| `--root` | — | Scan for condition folders |
| `--dirs` | — | Explicit condition directories |
| `--x` | `day` | `day` or `tick` |
| `--compare` / `--no-compare` | on | Overlay conditions |

---

## G. Paper figure bundles

### `plot_tier1_paper_figures.py`

**Plots:** Tier-1 bundle — child TTD with passive/active bounds, headroom heatmaps, TTD stats, PCA/LDA signatures at emergent stage.

| Argument | Default | Description |
|----------|---------|-------------|
| `--passive-csv` | required | Passive survival replicates |
| `--active-csv` | required | Active reference replicates |
| `--stage-csv` | repeatable | `label=conditional_metrics_<stage>.csv` |
| `--max-ticks` | 1000 | Normalization horizon |
| `--out-dir` | required | Output |

**Requires:** scikit-learn for PCA/LDA.

---

### `plot_tier2_paper_figures.py`

**Plots:** Tier-2 bundle — transfer plots, plasticity TTD trajectory, behavior–outcome scatter, world parameter table.

| Argument | Default | Description |
|----------|---------|-------------|
| `--out-dir` | `paper_figures_tier2` | Output |
| `--base` | `FinalGenomeRollouts_normal_seen` | Base world |
| `--unseen` | food30 + hard | Repeatable `label=path` |
| `--no-transfer` | off | Skip transfer |
| `--units` / `--day-step` / `--bootstrap` | | TTD scaling |
| `--stage-csv` | StageRollouts defaults | For plasticity summary |
| `--pca-features-csv` | tier1 output | PCA features |
| `--emergent-conditional-csv` | StageRollouts emergent | Behavior–outcome |

---

## H. Example: new rollout world end-to-end

```bash
# 1) Rollouts (see run_headless_rollout_emergence_final_genomes.sh)
./run_headless_rollout_emergence_final_genomes.sh \
  --genome-glob "Emergence_results/normal/*/final_genome.json" \
  --world normal --food-spawn-interval 30 \
  --out-root "FinalGenomeRollouts_my_food30"

# 2) TTD per genome
for d in FinalGenomeRollouts_my_food30/*/; do
  python3 plot_runlog_ttd_across_seeds.py --runlog-dir "$d"
done

# 3) Extended conditionals
python3 plot_runlog_conditional_behavior.py \
  --root FinalGenomeRollouts_my_food30 --extended

# 4) Figures
python3 plot_conditional_motivation_stacks.py \
  --from-csv FinalGenomeRollouts_my_food30/figures_conditional_extended/conditional_metrics_single.csv \
  --out-dir FinalGenomeRollouts_my_food30/figures_stacks

python3 plot_child_terminal_risk_profile.py \
  --root FinalGenomeRollouts_my_food30
```

---

## I. Script index (alphabetical)

| Script | Primary question |
|--------|------------------|
| `plot_behavior_surface_ttd.py` | How does TTD vary over a 2D behavior surface? |
| `plot_baldwin_*` | Sweep / Baldwin experiment grids & heatmaps |
| `plot_checkpoint_weights.py` | One run’s checkpoint weight evolution |
| `plot_child_terminal_risk_profile.py` | Child state at death by mother init |
| `plot_conditional_motivation_stacks.py` | P(sel\|mask) stacks for all masks |
| `plot_conditional_motivation_world_delta.py` | Unseen − base Δ heatmaps |
| `plot_conditions.py` | Passive baseline survival curves |
| `plot_emergence_checkpoint_weights_mean_heatmap.py` | Mean weights across emergence runs |
| `plot_emergence_summary.py` | Emergence lineage summary & grids |
| `plot_finalgenome_rollout_ttd_emergence_style.py` | TTD boxplots (rollout root) |
| `plot_init_sensitivity_signature.py` | Init sensitivity profiles & contrast |
| `plot_init_signature_food_threat.py` | Signatures on food×threat plane |
| `plot_joint_state_motivation_heatmap.py` | P(sel\|state bin) heatmaps |
| `plot_lineage_generations.py` | Single lineage CSV |
| `plot_lineage_weights_overlay.py` | Weight overlays across seeds |
| `plot_logged_run.py` | One run time series |
| `plot_logged_run_aggregate.py` | Multi-seed aggregate for one genome |
| `plot_rollout_ttd_emergence_full_set.py` | Full TTD figure set |
| `plot_runlog_conditional_behavior.py` | Core + extended conditional metrics |
| `plot_runlog_motivation_fractions_alive.py` | Unconditional motivation fractions |
| `plot_runlog_ttd_across_seeds.py` | TTD across rollout seeds |
| `plot_state_motivation_correlations.py` | Spearman state–motivation |
| `plot_tier1_paper_figures.py` | Paper tier-1 bundle |
| `plot_tier2_paper_figures.py` | Paper tier-2 bundle |
| `plot_threat_schedule.py` | Threat-schedule sweep plots |
| `plot_unseen_world_transfer.py` | TTD transfer base → unseen |
| `plot_weight_drift.py` | Plastic weight drift diagnostics |
| `plot_world_condition_comparison.py` | Multi-world conditional comparison |

---

*Generated for the Senior_thesis repo. Run any script with `python3 <script>.py --help` for the authoritative argument list.*
