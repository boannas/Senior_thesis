"""
Experiment: plasticity comparison (Fixed vs Outcome vs Hebbian).
Scenario (n_mothers, n_children, n_threats, food/day, etc.) is read from func/experiment_config.yaml.
Saves: summary and timeseries CSV with names including scenario_name.
"""
import sys
import random
import csv
import yaml
from pathlib import Path
import numpy as np

# Add project root
sys.path.insert(0, str(Path(__file__).resolve().parent))

from core.config.config import load_config, random_unique_positions
from core.world import World
from core.policies.mother import overall_deficit

# --- Experiment parameters (conditions stay here; scenario in YAML) ---
# Each entry: (condition_label, plasticity_rule, eta). Label must be unique.
CONDITIONS = [
    ("fixed",   None, None),
    ("outcome_eta0.001", "outcome", 0.001),
    ("outcome_eta0.01", "outcome", 0.01),
    ("outcome_eta0.1", "outcome", 0.1),
    ("hebbian_eta0.001", "hebbian", 0.001),
    ("hebbian_eta0.01", "hebbian", 0.01),
    ("hebbian_eta0.1", "hebbian", 0.1),
]

OUTPUT_DIR = Path(__file__).resolve().parent / "func"
EXPERIMENT_CONFIG_PATH = OUTPUT_DIR / "experiment_config.yaml"


def load_experiment_config():
    """Load scenario from func/experiment_config.yaml. Missing file or keys use defaults."""
    defaults = {
        "scenario_name": "baseline",
        "n_mothers": 1,
        "n_children": 1,
        "n_threats": 1,
        "n_food_initial": 0,
        "food_spawn_interval": None,
        "food_spawn_n": 1,
        "max_tick": 1000,
        "n_replicates": 30,
        "log_timeseries_interval": 10,
        "day_step": None,  # None = use base.yaml
    }
    if not EXPERIMENT_CONFIG_PATH.is_file():
        return defaults
    with open(EXPERIMENT_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    merged = dict(defaults)
    for k in defaults:
        if k in data:
            merged[k] = data[k]
    return merged


def run_one(plasticity_rule, seed, run_id, condition_label, scenario, eta=None, log_timeseries=False, step_interval=10):
    """
    Run one replicate. Scenario dict: n_mothers, n_children, n_threats, n_food_initial,
    food_spawn_interval, food_spawn_n, max_tick, day_step.
    Returns dict: mother_survival_tick, child_survival_tick, mean_deficit, ...
    """
    cfg = load_config(str(Path(__file__).parent / "core" / "base.yaml"))
    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"]
    dt = cfg.get("simulation", {}).get("dt", 0.05)
    day_step = scenario.get("day_step") or cfg.get("days", {}).get("ticks_per_day") or 100
    max_tick = scenario.get("max_tick", 1000)
    n_mothers = scenario.get("n_mothers", 1)
    n_children = scenario.get("n_children", 1)
    n_threats = scenario.get("n_threats", 1)
    n_food_initial = scenario.get("n_food_initial", 0)
    food_spawn_interval = scenario.get("food_spawn_interval")
    food_spawn_n = scenario.get("food_spawn_n", 1)

    random.seed(seed)
    np.random.seed(seed)
    occupied = set()
    mother_starts, occupied = random_unique_positions(n_mothers, grid_w, grid_h, occupied)
    child_starts,  occupied = random_unique_positions(n_children, grid_w, grid_h, occupied)
    threat_starts, occupied = random_unique_positions(n_threats, grid_w, grid_h, occupied)
    food_positions, _       = random_unique_positions(n_food_initial, grid_w, grid_h, occupied)

    world = World(
        grid_w, grid_h,
        mother_starts=mother_starts,
        child_start=child_starts,
        food_positions=food_positions,
        threat_starts=threat_starts,
        seed=seed,
        day_step=day_step,
        plasticity_rule=plasticity_rule,
        food_spawn_interval=food_spawn_interval,
        food_spawn_n=food_spawn_n,
    )

    mother = world.mothers[0]
    child = world.children[0]
    if eta is not None:
        for m in world.mothers:
            m.eta = eta
    mother_survival_tick = None
    child_survival_tick = None
    deficits = []
    timeseries_rows = []
    mot_counts = {"Forage": 0, "Care": 0, "Self": 0, "Protect": 0}

    for step in range(int(max_tick)):
        world.step(dt)
        sel = getattr(mother, "selected_motivation", None)
        if sel in mot_counts:
            mot_counts[sel] += 1
        elif sel is not None:
            mot_counts[sel] = mot_counts.get(sel, 0) + 1

        if mother_survival_tick is None and not mother.is_alive():
            mother_survival_tick = world.tick
        if child_survival_tick is None and not child.is_alive():
            child_survival_tick = world.tick

        try:
            d = overall_deficit(mother)
            deficits.append(d)
        except Exception:
            d = float("nan")
        if log_timeseries and step_interval > 0 and (world.tick % step_interval == 0 or step == 0):
            row = {
                "condition": condition_label,
                "run_id": run_id,
                "tick": world.tick,
                "mother_alive": 1 if mother.is_alive() else 0,
                "child_alive": 1 if child.is_alive() else 0,
                "mother_energy": getattr(mother, "energy", float("nan")),
                "child_hunger": getattr(child, "hunger", float("nan")),
                "child_warmth": getattr(child, "warmth", float("nan")),
                "deficit": d,
                "eta": eta if eta is not None else "",
                "selected_motivation": sel if sel is not None else "",
            }
            timeseries_rows.append(row)

        if mother_survival_tick is not None and child_survival_tick is not None:
            break

    if mother_survival_tick is None:
        mother_survival_tick = world.tick
    if child_survival_tick is None:
        child_survival_tick = world.tick

    total_steps = sum(mot_counts.values()) or 1
    mot_props = {k: mot_counts.get(k, 0) / total_steps for k in ("Forage", "Care", "Self", "Protect")}
    mean_deficit = np.mean(deficits) if deficits else float("nan")
    return {
        "mother_survival_tick": mother_survival_tick,
        "child_survival_tick": child_survival_tick,
        "mean_deficit": mean_deficit,
        "final_mother_energy": getattr(mother, "energy", float("nan")),
        "final_child_hunger": getattr(child, "hunger", float("nan")),
        "final_child_warmth": getattr(child, "warmth", float("nan")),
        "mot_props": mot_props,
        "timeseries_rows": timeseries_rows,
    }


def main():
    scenario = load_experiment_config()
    scenario_name = scenario.get("scenario_name", "baseline")
    max_tick = int(scenario.get("max_tick", 1000))
    n_replicates = int(scenario.get("n_replicates", 30))
    log_interval = int(scenario.get("log_timeseries_interval", 10))
    log_timeseries = log_interval > 0

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    summary_path = OUTPUT_DIR / f"experiment_plasticity_{scenario_name}_summary.csv"
    timeseries_path = OUTPUT_DIR / f"experiment_plasticity_{scenario_name}_timeseries.csv"

    print("Scenario:", scenario_name)
    print("  n_mothers={}, n_children={}, n_threats={}, n_food_initial={}".format(
        scenario.get("n_mothers", 1), scenario.get("n_children", 1),
        scenario.get("n_threats", 1), scenario.get("n_food_initial", 0)))
    print("  food_spawn_interval={}, food_spawn_n={}".format(scenario.get("food_spawn_interval"), scenario.get("food_spawn_n", 1)))
    print("  max_tick={}, n_replicates={}".format(max_tick, n_replicates))
    print("Summary CSV:", summary_path)
    if log_timeseries:
        print("Timeseries CSV (every {} ticks):".format(log_interval), timeseries_path)
    print()

    summary_rows = []
    timeseries_rows = []
    results = {}

    for item in CONDITIONS:
        if len(item) == 3:
            label, plasticity_rule, eta = item
        else:
            label, plasticity_rule = item[0], item[1]
            eta = None
        mother_ticks = []
        child_ticks = []
        for run in range(n_replicates):
            seed = 1000 + run
            out = run_one(
                plasticity_rule, seed, run_id=run, condition_label=label, scenario=scenario, eta=eta,
                log_timeseries=log_timeseries, step_interval=log_interval,
            )
            mother_ticks.append(out["mother_survival_tick"])
            child_ticks.append(out["child_survival_tick"])
            mp = out.get("mot_props", {})
            row = {
                "condition": label,
                "plasticity_rule": plasticity_rule if plasticity_rule else "fixed",
                "eta": eta if eta is not None else "",
                "run_id": run,
                "seed": seed,
                "mother_survival_tick": out["mother_survival_tick"],
                "child_survival_tick": out["child_survival_tick"],
                "mean_deficit": out["mean_deficit"],
                "final_mother_energy": out["final_mother_energy"],
                "final_child_hunger": out["final_child_hunger"],
                "final_child_warmth": out["final_child_warmth"],
                "p_Forage": mp.get("Forage", 0),
                "p_Care": mp.get("Care", 0),
                "p_Self": mp.get("Self", 0),
                "p_Protect": mp.get("Protect", 0),
            }
            summary_rows.append(row)
            if out.get("timeseries_rows"):
                timeseries_rows.extend(out["timeseries_rows"])
        results[label] = {
            "mother_survival_tick": np.array(mother_ticks),
            "child_survival_tick": np.array(child_ticks),
        }

    # Write summary CSV
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=[
            "condition", "plasticity_rule", "eta", "run_id", "seed",
            "mother_survival_tick", "child_survival_tick",
            "mean_deficit", "final_mother_energy", "final_child_hunger", "final_child_warmth",
            "p_Forage", "p_Care", "p_Self", "p_Protect",
        ])
        w.writeheader()
        w.writerows(summary_rows)
    print("Wrote", summary_path)

    # Write timeseries CSV
    if timeseries_rows:
        with open(timeseries_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=[
                "condition", "run_id", "tick", "mother_alive", "child_alive",
                "mother_energy", "child_hunger", "child_warmth", "deficit", "eta",
                "selected_motivation",
            ])
            w.writeheader()
            w.writerows(timeseries_rows)
        print("Wrote", timeseries_path)

    # Report mean ± SD
    print()
    print("=" * 60)
    print("Mother survival tick (higher = better; max = {})".format(max_tick))
    print("=" * 60)
    for item in CONDITIONS:
        label = item[0]
        arr = results[label]["mother_survival_tick"]
        mean, sd = np.mean(arr), np.std(arr, ddof=1) if len(arr) > 1 else 0.0
        print("  {:20s}: {:.1f} ± {:.1f}".format(label, mean, sd))
    print()
    print("=" * 60)
    print("Child survival tick (higher = better; max = {})".format(max_tick))
    print("=" * 60)
    for item in CONDITIONS:
        label = item[0]
        arr = results[label]["child_survival_tick"]
        mean, sd = np.mean(arr), np.std(arr, ddof=1) if len(arr) > 1 else 0.0
        print("  {:20s}: {:.1f} ± {:.1f}".format(label, mean, sd))
    print()
    print("Done. Run plot_experiment_plasticity.py to plot.")


if __name__ == "__main__":
    main()
