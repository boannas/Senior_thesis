# Run: emergence normal batch (seeds 47–51)

This batch runs **45** `run_evolve_lineage.py` jobs: **E2 / P1 / P2** × three init modes × seeds **47–51**, writing under `Emergence_results/normal/`.

Script: `run_emergence_normal_seeds47_51.sh` (repository root, next to `run_evolve_lineage.py`).

## Prerequisites

- Shell: **bash**
- Working directory: repository root (the script `cd`s to its own directory, so you can invoke it from anywhere with a path to the script).
- Python: `python3` on `PATH`, with the environment your project uses for `run_evolve_lineage.py`.

<!-- ## Sequential (all 45 runs, one after another)

```bash
cd /path/to/Senior_thesis   # e.g. cd /home/fibo1/foo_thesis/Senior_thesis
./run_emergence_normal_seeds47_51.sh
``` -->

## Parallel across 12 terminals (disjoint shards)

Each `WORKER` value **0 … 11** runs a **round-robin** subset; together they cover every job exactly once.

Open **12** terminals and run one command per terminal:

```bash
cd /path/to/Senior_thesis   # e.g. cd /home/fibo1/foo_thesis/Senior_thesis

# WORKER=0  ./run_emergence_normal_seeds47_51.sh


WORKER=1  ./run_emergence_normal_seeds47_51.sh
WORKER=2  ./run_emergence_normal_seeds47_51.sh
WORKER=3  ./run_emergence_normal_seeds47_51.sh
WORKER=4  ./run_emergence_normal_seeds47_51.sh
WORKER=5  ./run_emergence_normal_seeds47_51.sh
WORKER=6  ./run_emergence_normal_seeds47_51.sh
WORKER=7  ./run_emergence_normal_seeds47_51.sh
WORKER=8  ./run_emergence_normal_seeds47_51.sh
WORKER=9  ./run_emergence_normal_seeds47_51.sh
WORKER=10 ./run_emergence_normal_seeds47_51.sh
WORKER=11 ./run_emergence_normal_seeds47_51.sh
```

`WORKER` must be an integer in **0 … 11**. If it is missing, the script runs **all** jobs (sequential).

## Outputs

Runs use tags `E2_*`, `P1_*`, `P2_*` with init name and seed, for example:

`Emergence_results/normal/E2_anti_maternal_seed47`

## Stopping on errors

The script uses `set -euo pipefail`: the first failing `python3` invocation exits the script with a non-zero status.
