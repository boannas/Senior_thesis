# Simulation of Maternal Instinct Baseline: README

## Installation & Requirements

Ensure you have Python 3 installed. Required dependencies typically include:
```bash
pip install numpy pandas matplotlib seaborn pygame
```

## Usage Guide

### 1. Interactive Visual Simulation
To launch the real-time Pygame gridworld visualization:
```bash
python run_gridworld.py
```
*(Performance logs for interactive runs are saved to `run_log.csv`)*

### 2. Running Headless Baseline Experiments
To generate the core baseline datasets (bypassing the GUI for speed with batch replicates):
```bash
python baseline_0_runner.py
python baseline_1_runner.py
```
*Outputs are written directly to `test_results/baseline_0/` and `test_results/baseline_1/`.*

### 3. Structural Validation Scripts
To guarantee the scientific integrity of the motivation dynamics, several isolated edge-case validators are provided:
```bash
python validation_case2_no_food.py       # Confirms starvation tracks properly
python validation_case3_no_care.py       # Confirms thermal deficits track properly
python validation_case4_stability.py     # Cross-seed reproducibility check
python validation_case6_bifurcation.py   # Stability tests across unstable equilibrium points
```

### 4. Data Visualization & Analytics
Plot diagnostics on single standard logged runs:
```bash
python plot_logged_run.py
```
Plotting tools and baseline statistical comparisons are typically exported directly to the `plots/` directory via analytics scripts (e.g., `baseline1_distribution.png`).

## Documentation & Design
For in-depth analysis of the system architecture, validation results, limits, and the methodology behind the baseline selection, please refer to the primary academic report:
- `Experiment_Baseline_Report.md` - Backgound, Assumption, Key Insight
- `Design_Baseline_Report.md` - Design, Parameters
