# Maternal-Intuition Gridworld (quick usage)


## Run interactive gridworld (pygame)

```bash
python run_gridworld.py
```

- Logs a CSV to `run_log.csv` (see `func/run_logger.py`).

## Run baseline survival test (headless)

```bash
python test_baseline_survival.py
```

- Saves diagnostic plots under `test_results/`.


## Run experiment (batch runs, headless)

```bash
python run_experiment_plasticity.py
```

- Scenario settings live in `func/experiment_config.yaml`.
- Outputs are written under `func/` (summary + timeseries CSV).

## Plot graphs
`this close to test_bestline_survival.py plot`
Plot a single run log:

```bash
python plot_logged_run.py
```

Plot experiment outputs:

```bash
python plot_experiment_plasticity.py  --scenario {scenario_name}
```
- Scenario_name live in `func/experiment_config.yaml`.


