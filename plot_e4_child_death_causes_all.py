#!/usr/bin/env python3
"""
plot_e4_child_death_causes_all.py
==================================
100% stacked-bar plots for *child cause of death* across the 9 E4 unseen
conditions, with optional breakdown by initialization regime.

Per-condition CSVs expected (built by plot_child_death_causes.py):
  <out-dir>/<world>_f<food>/child_death_causes_per_run.csv

Outputs written to <out-dir>:
  E4_child_death_causes_by_condition.csv          (aggregate over all inits)
  E4_child_death_causes_by_condition.png          (aggregate plot)
  E4_child_death_causes_by_init_anti_maternal.png
  E4_child_death_causes_by_init_random_uniform.png
  E4_child_death_causes_by_init_pro_maternal.png
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from typing import Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


CAUSE_ORDER = ["alive", "injury", "hunger", "cold", "heat", "unknown_nonfinite", "unknown_missing"]
CAUSE_COLORS = {
    "alive": "#7f7f7f",
    "injury": "#9467bd",
    "hunger": "#e15759",
    "cold": "#4e79a7",
    "heat": "#f28e2b",
    "unknown_nonfinite": "#9c755f",
    "unknown_missing": "#bab0ac",
}

WORLDS = ("easy", "normal", "hard")
FOODS = (5, 15, 25)
INIT_ORDER = ("anti_maternal", "random_uniform", "pro_maternal")
INIT_LABELS = {
    "anti_maternal": "anti-maternal",
    "random_uniform": "random",
    "pro_maternal": "pro-maternal",
}


def _cond_label(world: str, food: int) -> str:
    return f"{world}_f{food}"


def _cond_xtick_label(world: str, food: int) -> str:
    # Make it compact for rotated ticks.
    return f"{world}\nf{food}"


def _maybe_build_condition(
    *,
    repo_root: str,
    world: str,
    food: int,
    out_dir: str,
    ensure: bool,
    plot_child_death_causes_py: str,
) -> str:
    root = os.path.join(repo_root, f"E4_{world}_{food}")
    cond_tag = _cond_label(world, food)
    cond_out = os.path.join(out_dir, cond_tag)
    os.makedirs(cond_out, exist_ok=True)
    run_csv = os.path.join(cond_out, "child_death_causes_per_run.csv")
    if (not ensure) and os.path.isfile(run_csv):
        return run_csv

    cmd = [
        sys.executable,
        plot_child_death_causes_py,
        "--root",
        root,
        "--out-dir",
        cond_out,
    ]
    print("[info]", " ".join(cmd))
    p = subprocess.run(cmd, check=False)
    if p.returncode != 0:
        raise RuntimeError(f"plot_child_death_causes failed for {cond_tag} (exit={p.returncode})")
    if not os.path.isfile(run_csv):
        raise RuntimeError(f"Missing expected CSV after build: {run_csv}")
    return run_csv


def _load_all_runs(run_csvs: dict[str, str], condition_order: list[str]) -> pd.DataFrame:
    """Concatenate all per-condition run CSVs into one DataFrame with a 'condition' column."""
    parts = []
    for cond in condition_order:
        df = pd.read_csv(run_csvs[cond])
        if "death_cause" not in df.columns:
            raise RuntimeError(f"death_cause column missing from {run_csvs[cond]}")
        df["condition"] = cond
        parts.append(df)
    return pd.concat(parts, ignore_index=True)


def _cause_fracs(df: pd.DataFrame) -> dict[str, float]:
    total = len(df)
    if total == 0:
        return {f"frac_{c}": 0.0 for c in CAUSE_ORDER}
    return {f"frac_{c}": int(df["death_cause"].eq(c).sum()) / total for c in CAUSE_ORDER}


def _summarize_by_group(
    all_runs: pd.DataFrame,
    condition_order: list[str],
    *,
    init_filter: str | None = None,
) -> pd.DataFrame:
    """Summarize cause fractions per condition, optionally filtered by init."""
    rows = []
    sub = all_runs if init_filter is None else all_runs[all_runs["init"] == init_filter]
    for cond in condition_order:
        cdf = sub[sub["condition"] == cond]
        row: dict = {"condition": cond, "n_runs": len(cdf)}
        row.update(_cause_fracs(cdf))
        rows.append(row)
    return pd.DataFrame(rows)


def _stacked_bars_on_ax(
    ax: plt.Axes,
    summary_df: pd.DataFrame,
    condition_order: list[str],
    *,
    title: str,
) -> None:
    x = np.arange(len(condition_order), dtype=float)
    bottom = np.zeros(len(condition_order), dtype=float)

    for cause in CAUSE_ORDER:
        vals = []
        for c in condition_order:
            row = summary_df.loc[summary_df["condition"] == c]
            vals.append(float(row.iloc[0][f"frac_{cause}"]) if not row.empty else 0.0)
        vals_arr = np.nan_to_num(np.array(vals, dtype=float), nan=0.0)
        ax.bar(x, vals_arr, bottom=bottom, color=CAUSE_COLORS.get(cause, "#999999"),
               linewidth=0.0, label=cause)
        bottom += vals_arr

    labels = [_cond_xtick_label(*cond.split("_f", 1)) for cond in condition_order]
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0, fontsize=8)
    ax.set_ylim(0.0, 1.0)
    ax.set_ylabel("fraction of runs (100% stacked)")
    ax.set_title(title, fontsize=11)
    ax.grid(True, axis="y", color="0.92")


def plot_stacked(
    summary_df: pd.DataFrame,
    *,
    condition_order: list[str],
    title: str = "Child death causes — E4 unseen conditions",
    out_png: str,
) -> None:
    """Single-panel stacked-bar figure."""
    fig, ax = plt.subplots(1, 1, figsize=(14.2, 5.2))
    _stacked_bars_on_ax(ax, summary_df, condition_order, title=title)
    ax.legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)

def _split_condition(cond: str) -> tuple[str, int]:
    w, fstr = cond.split("_f", 1)
    return w, int(fstr)


def _summary_table(all_runs: pd.DataFrame) -> pd.DataFrame:
    """
    Long-form summary: one row per (condition, world, food, plasticity, init).
    Columns include counts and fractions for each cause.
    """
    x = all_runs.copy()
    if "condition" not in x.columns:
        raise RuntimeError("all_runs missing 'condition'")
    if "plasticity" not in x.columns or "init" not in x.columns:
        raise RuntimeError("all_runs missing 'plasticity' or 'init'")
    if "death_cause" not in x.columns:
        raise RuntimeError("all_runs missing 'death_cause'")

    # parse world/food from condition for convenience
    world_food = x["condition"].astype(str).apply(_split_condition)
    x["world"] = [wf[0] for wf in world_food]
    x["food"] = [wf[1] for wf in world_food]

    keys = ["condition", "world", "food", "plasticity", "init"]
    out = x.groupby(keys, as_index=False).agg(n_runs=("death_cause", "count"))

    for cause in CAUSE_ORDER:
        out[f"n_{cause}"] = (
            x.assign(_is=x["death_cause"].eq(cause).astype(int))
            .groupby(keys)["_is"]
            .sum()
            .to_numpy()
        )
        out[f"frac_{cause}"] = out[f"n_{cause}"] / out["n_runs"].replace(0, np.nan)
        out[f"frac_{cause}"] = out[f"frac_{cause}"].fillna(0.0)

    # dominant cause (excluding alive optionally)
    frac_cols = [f"frac_{c}" for c in CAUSE_ORDER]
    out["dominant_cause"] = out[frac_cols].idxmax(axis=1).str.replace("frac_", "", regex=False)
    out["dominant_frac"] = out[frac_cols].max(axis=1)

    # stable ordering
    out["plasticity"] = pd.Categorical(out["plasticity"], categories=["E2", "P1", "P2"], ordered=True)
    out["init"] = pd.Categorical(out["init"], categories=list(INIT_ORDER), ordered=True)
    out["world"] = pd.Categorical(out["world"], categories=list(WORLDS), ordered=True)
    out["food"] = pd.Categorical(out["food"], categories=list(FOODS), ordered=True)
    out = out.sort_values(["init", "plasticity", "world", "food"]).reset_index(drop=True)
    return out


def _trend_table(summary_long: pd.DataFrame, *, group: str) -> pd.DataFrame:
    """
    Aggregate trends by either 'world' (collapsing food) or 'food' (collapsing world),
    keeping init × plasticity.
    """
    if group not in {"world", "food"}:
        raise ValueError("group must be 'world' or 'food'")
    keys = [group, "plasticity", "init"]

    # Weighted average across the collapsed axis using n_runs (avoid giving tiny groups same weight).
    def _wavg(df: pd.DataFrame, col: str) -> float:
        w = df["n_runs"].to_numpy(dtype=float)
        y = df[col].to_numpy(dtype=float)
        if w.sum() <= 0:
            return float("nan")
        return float(np.sum(w * y) / np.sum(w))

    rows = []
    for (gval, plast, init), sub in summary_long.groupby(keys):
        row: dict[str, object] = {group: gval, "plasticity": plast, "init": init, "n_runs": int(sub["n_runs"].sum())}
        for cause in CAUSE_ORDER:
            row[f"frac_{cause}"] = _wavg(sub, f"frac_{cause}")
        # dominant cause after aggregation
        frac_cols = [f"frac_{c}" for c in CAUSE_ORDER]
        dom_idx = int(np.nanargmax([row[c] for c in frac_cols]))
        row["dominant_cause"] = CAUSE_ORDER[dom_idx]
        row["dominant_frac"] = float([row[c] for c in frac_cols][dom_idx])
        rows.append(row)

    out = pd.DataFrame(rows)
    out["plasticity"] = pd.Categorical(out["plasticity"], categories=["E2", "P1", "P2"], ordered=True)
    out["init"] = pd.Categorical(out["init"], categories=list(INIT_ORDER), ordered=True)
    if group == "world":
        out["world"] = pd.Categorical(out["world"], categories=list(WORLDS), ordered=True)
        out = out.sort_values(["init", "plasticity", "world"]).reset_index(drop=True)
    else:
        out["food"] = pd.Categorical(out["food"], categories=list(FOODS), ordered=True)
        out = out.sort_values(["init", "plasticity", "food"]).reset_index(drop=True)
    return out


def _write_tables(all_runs: pd.DataFrame, *, out_dir: str) -> None:
    """
    Write CSVs meant to be read quickly:
      - long table (condition × init × plasticity)
      - trends collapsed by world / by food
      - compact 'dominant cause' views
    """
    long_df = _summary_table(all_runs)
    long_path = os.path.join(out_dir, "E4_child_death_causes_long__condition_init_plasticity.csv")
    long_df.to_csv(long_path, index=False)
    print("[ok] wrote", long_path)

    by_world = _trend_table(long_df, group="world")
    by_world_path = os.path.join(out_dir, "E4_child_death_causes_trends__by_world_init_plasticity.csv")
    by_world.to_csv(by_world_path, index=False)
    print("[ok] wrote", by_world_path)

    by_food = _trend_table(long_df, group="food")
    by_food_path = os.path.join(out_dir, "E4_child_death_causes_trends__by_food_init_plasticity.csv")
    by_food.to_csv(by_food_path, index=False)
    print("[ok] wrote", by_food_path)

    # Compact dominant-cause tables (easier for first-time readers)
    dom_cols = ["init", "plasticity", "world", "food", "n_runs", "dominant_cause", "dominant_frac"]
    dom_cond = long_df[dom_cols].copy()
    dom_cond_path = os.path.join(out_dir, "E4_child_death_causes_dominant__per_condition.csv")
    dom_cond.to_csv(dom_cond_path, index=False)
    print("[ok] wrote", dom_cond_path)

    dom_world_cols = ["init", "plasticity", "world", "n_runs", "dominant_cause", "dominant_frac"]
    dom_world = by_world[dom_world_cols].copy()
    dom_world_path = os.path.join(out_dir, "E4_child_death_causes_dominant__by_world.csv")
    dom_world.to_csv(dom_world_path, index=False)
    print("[ok] wrote", dom_world_path)

    dom_food_cols = ["init", "plasticity", "food", "n_runs", "dominant_cause", "dominant_frac"]
    dom_food = by_food[dom_food_cols].copy()
    dom_food_path = os.path.join(out_dir, "E4_child_death_causes_dominant__by_food.csv")
    dom_food.to_csv(dom_food_path, index=False)
    print("[ok] wrote", dom_food_path)


def plot_stacked_by_init(
    all_runs: pd.DataFrame,
    *,
    condition_order: list[str],
    out_dir: str,
) -> None:
    """One figure per init (3 panels: E2 | P1 | P2 columns × conditions on x-axis).

    Layout:  rows = init groups (anti / random / pro)
             each row = single stacked-bar panel (all conditions)
    """
    from matplotlib.gridspec import GridSpec

    PLAST_ORDER = ("E2", "P1", "P2")
    PLAST_TITLES = {"E2": "E2 (no plasticity)", "P1": "P1", "P2": "P2"}

    for init in INIT_ORDER:
        init_label = INIT_LABELS.get(init, init)
        fig = plt.figure(figsize=(15.0, 5.5), constrained_layout=True)
        gs = GridSpec(1, 3, figure=fig, wspace=0.25)
        axes = [fig.add_subplot(gs[0, i]) for i in range(3)]

        for ax, plast in zip(axes, PLAST_ORDER):
            sub = all_runs[(all_runs["init"] == init) & (all_runs["plasticity"] == plast)]
            summ = _summarize_by_group(sub.assign(condition=sub["condition"]),
                                       condition_order)
            _stacked_bars_on_ax(ax, summ, condition_order, title=PLAST_TITLES.get(plast, plast))
            if ax is not axes[0]:
                ax.set_ylabel("")

        # shared legend below
        handles = [
            plt.Rectangle((0, 0), 1, 1, color=CAUSE_COLORS[c], label=c)
            for c in CAUSE_ORDER
        ]
        fig.legend(handles=handles, loc="lower center", ncol=len(CAUSE_ORDER),
                   frameon=False, fontsize=9, bbox_to_anchor=(0.5, -0.08))
        fig.suptitle(f"Child death causes — {init_label} | E4 unseen conditions",
                     fontsize=12)
        out_png = os.path.join(out_dir, f"E4_child_death_causes_by_init_{init}.png")
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print("[ok] wrote", out_png)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo-root", default=".", help="Repo root containing E4_* folders")
    ap.add_argument(
        "--out-dir",
        default="E4_figures/death_causes",
        help="Where per-condition death-cause outputs live (and where combined outputs are written)",
    )
    ap.add_argument("--ensure", action="store_true", help="Build missing per-condition CSVs if needed")
    ap.add_argument(
        "--plot-only",
        action="store_true",
        help="Do not build; fail if any per-condition CSV is missing",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    repo_root = os.path.abspath(args.repo_root)
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    plot_child_death_causes_py = os.path.join(os.path.dirname(__file__), "plot_child_death_causes.py")

    condition_order: list[str] = [_cond_label(w, f) for w in WORLDS for f in FOODS]

    # Build/read per-condition run CSVs.
    run_csvs: dict[str, str] = {}
    for w in WORLDS:
        for f in FOODS:
            cond = _cond_label(w, f)
            cond_out_dir = os.path.join(out_dir, cond)
            run_csv = os.path.join(cond_out_dir, "child_death_causes_per_run.csv")

            if os.path.isfile(run_csv):
                run_csvs[cond] = run_csv
                continue

            if args.plot_only:
                raise RuntimeError(f"Missing per-condition CSV and --plot-only set: {run_csv}")
            if not args.ensure:
                raise RuntimeError(
                    f"Missing per-condition CSV: {run_csv}\n"
                    f"Re-run with --ensure to build missing condition outputs."
                )
            run_csvs[cond] = _maybe_build_condition(
                repo_root=repo_root,
                world=w,
                food=f,
                out_dir=out_dir,
                ensure=True,
                plot_child_death_causes_py=plot_child_death_causes_py,
            )

    # Load all runs into one DataFrame.
    all_runs = _load_all_runs(run_csvs, condition_order)

    # Aggregate summary (all inits combined).
    summary_df = _summarize_by_group(all_runs, condition_order)
    csv_out = os.path.join(out_dir, "E4_child_death_causes_by_condition.csv")
    summary_df.to_csv(csv_out, index=False)
    print("[ok] wrote", csv_out)

    # Aggregate plot.
    plot_stacked(
        summary_df,
        condition_order=condition_order,
        title="Child death causes — E4 unseen conditions (all inits)",
        out_png=os.path.join(out_dir, "E4_child_death_causes_by_condition.png"),
    )

    # Per-init plots (anti / random / pro), split by plasticity panels.
    plot_stacked_by_init(all_runs, condition_order=condition_order, out_dir=out_dir)

    # Easy-to-read summary tables (init × plasticity × condition/world/food).
    _write_tables(all_runs, out_dir=out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

