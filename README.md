# Maternal-instuition simulation

Gridworld simulation with mother–child–threat dynamics, internal states, motivation selection, optional plasticity, and single-lineage evolution of motivation weights.

**Thesis report:** [AJ.Blink — Maternal Instinct (PDF)](Thesis_Report_Farao/AJ.Blink-Maternal_Instinct.pdf)

<!-- **Final defense:** [Final Defense (PDF)](Thesis_Report_Farao/Final-Defense-T_T.pdf) -->

**Final presentation slides:** [Canva deck](https://canva.link/ojzvlcfes79tz12)

## Setup

```bash
pip install numpy pandas matplotlib seaborn pygame
```

## Run

| What | Command |
|------|---------|
| Interactive viewer | `python run_gridworld.py` |
| Viewer with evolved genome | `python -m core.ui.pygame_viewer --genome path/to/final_genome.json` |
| Passive survival lower bound | `python baseline_passive_lower_bound.py` |
| Single-lineage evolution | `python run_evolve_lineage.py` |
| Headless rollout logging | `python headless_rollout_log.py --genome path/to/final_genome.json --seed 10000` |
| Compare plasticity rules | `python compare_plasticity_rules.py --genome path/to/genome.json` |

Outputs (CSVs, JSON genomes, figures) are written locally under `test_results/`, `result_experiment/`, or paths you pass with `--output-dir` / `--out-dir`. They are not committed to git.

## Plot

| What | Command |
|------|---------|
| Single run log | `python plot_logged_run.py run_log.csv` |
| Aggregate rollout logs | `python plot_logged_run_aggregate.py --out-dir path/to/runs` |
| Lineage generations | `python plot_lineage_generations.py --csv path/to/lineage_generations.csv` |
| Lineage weight overlay | `python plot_lineage_weights_overlay.py --root path/to/runs` |
| Baldwin grid | `python plot_baldwin_grid.py --root path/to/runs` |
| Baldwin full grid | `python plot_baldwin_full_grid.py --root path/to/runs` |
| Baldwin heatmap | `python plot_baldwin_heatmap.py --csv path/to/summary.csv` |
| Threat schedule | `python plot_threat_schedule.py --root path/to/runs` |
| Weight drift | `python plot_weight_drift.py --glob "path/to/run_*.csv"` |
| TTD across seeds | `python plot_runlog_ttd_across_seeds.py --root path/to/runs` |
| Motivation fractions (alive) | `python plot_runlog_motivation_fractions_alive.py --root path/to/runs` |
| Checkpoint weights | `python plot_checkpoint_weights.py --root path/to/checkpoints` |
| Emergence summary | `python plot_emergence_summary.py --root path/to/runs` |
| Summarize lineage runs (CSV for plots) | `python summarize_lineage_runs.py --root path/to/runs` |

Most plot scripts accept `--help` for required paths and options.

## Repo layout

- `core/` — world loop, agents, policies, movement, Pygame UI
- `func/` — logging, pathfinding, baseline weight config
- `run_*.py`, `baseline_passive_lower_bound.py`, `headless_rollout_log.py`, `compare_plasticity_rules.py` — runners
- `plot_*.py`, `summarize_lineage_runs.py` — plotting and aggregation
