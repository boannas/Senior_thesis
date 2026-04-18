# Maternal-instuition simulation (`evolve-lineage`)

Minimal gridworld with mother–child–threat dynamics, internal states, motivation selection, and optional plasticity. This branch focuses on **single-lineage evolution** (genes mutate across generations; plasticity resets per life)—implementation lives alongside existing baselines.

## Setup

```bash
pip install numpy pandas matplotlib seaborn pygame
```

## Run

| What | Command |
|------|---------|
| Interactive viewer | `python run_gridworld.py` |
| Baseline 0 (fixed 0.5 weights) | `python baseline_0_runner.py` |
| Baseline 1 (random weights) | `python baseline_1_runner.py` |
| Passive survival lower bound | `python baseline_passive_lower_bound.py` |

CSV summaries go to `test_results/` (local only; not committed).

## Repo layout

- `core/` — world loop, agents, policies (`mother.py`, `threat.py`), movement
- `func/` — helpers (logging, pathfinding, optional configs)
- `archive/` — old calibration / sweep scripts (reference only)

## Docs

- `Experiment_Baseline_Report.md`, `Design_Baseline_Report.md` — baseline methodology
