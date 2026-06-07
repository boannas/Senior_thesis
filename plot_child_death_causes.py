#!/usr/bin/env python3
"""
plot_child_death_causes.py
=========================
Visualize *how the child dies* from headless rollout logs (run_*.csv).

What this script produces
-------------------------
Given a root directory that contains rollout subdirectories like:

  <root>/
    E2_random_uniform_seed42/
      run_00000.csv
      run_00001.csv
      ...

It outputs, under --out-dir:

  - child_death_causes_per_run.csv
      One row per run_*.csv: alive/died + death tick + inferred death cause.

  - child_death_causes_per_genome.csv
      One row per (plasticity, init, evolve_seed): proportions over runs.

  - death_survival_stacked_by_genome.png
      100% stacked bars: Alive vs death causes, one bar per genome.

  - death_cause_mix_by_init.png
      100% stacked bars: Alive vs death causes, aggregated by (plasticity, init).

Death cause inference (mechanistic)
----------------------------------
We treat a child as "dead" at the first tick where any of the ChildAgent death
conditions would be true (see core/agents.py):

  - hunger >= 100
  - warmth <= 0  (cold)
  - warmth >= 100 (heat)
  - injury >= 100

We infer the *cause* as whichever threshold is crossed first in the log.
If multiple are crossed on the same tick, ties break in this order:
injury > hunger > cold > heat (arbitrary but consistent).

If the log uses NaNs to signal death (older convention), we treat the first
non-finite tick in any of {c0_hunger,c0_warmth,c0_injury} as death with cause
"unknown_nonfinite" unless a threshold cause occurs earlier.

Usage
-----
  python3 plot_child_death_causes.py \
      --root StageRollouts_normal/emergent \
      --out-dir StageRollouts_normal/figures_child_death_causes_emergent
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from dataclasses import dataclass

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

import plot_runlog_conditional_behavior as crb


INIT_ORDER = crb.INIT_ORDER
PLAST_ORDER = crb.PLAST_ORDER
RUN_RE = crb.RUN_RE

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


@dataclass(frozen=True)
class DeathInfo:
    died: bool
    death_tick: float  # 1-based tick index, NaN if unknown
    cause: str         # one of CAUSE_ORDER


def _first_index(mask: np.ndarray) -> int | None:
    if mask is None or mask.size == 0:
        return None
    if not mask.any():
        return None
    return int(np.argmax(mask))


def _infer_death_from_arrays(
    hunger: np.ndarray | None,
    warmth: np.ndarray | None,
    injury: np.ndarray | None,
) -> DeathInfo:
    # Missing columns: can't infer
    if hunger is None or warmth is None or injury is None:
        return DeathInfo(died=False, death_tick=float("nan"), cause="unknown_missing")

    h = hunger.astype(float, copy=False)
    w = warmth.astype(float, copy=False)
    i = injury.astype(float, copy=False)

    # Many of your logs keep writing after child death and mark child columns non-finite (NaN).
    # So: detect death tick from first non-finite row (if present), then infer cause from the
    # *last finite* child state right before that tick (or from earlier threshold crossings).

    finite_h = np.isfinite(h)
    finite_w = np.isfinite(w)
    finite_i = np.isfinite(i)

    idx_nonfinite = _first_index(~finite_h | ~finite_w | ~finite_i)

    # Threshold crossings while still finite (ChildAgent.is_alive)
    idx_injury = _first_index(finite_i & (i >= 100.0))
    idx_hunger = _first_index(finite_h & (h >= 100.0))
    idx_cold = _first_index(finite_w & (w <= 0.0))
    idx_heat = _first_index(finite_w & (w >= 100.0))

    threshold_candidates: list[tuple[int, str]] = []
    for idx, cause in (
        (idx_injury, "injury"),
        (idx_hunger, "hunger"),
        (idx_cold, "cold"),
        (idx_heat, "heat"),
    ):
        if idx is not None:
            threshold_candidates.append((idx, cause))

    if idx_nonfinite is None and not threshold_candidates:
        return DeathInfo(died=False, death_tick=float("nan"), cause="alive")

    # If a threshold crossing happens before the first non-finite tick (or there is no non-finite),
    # treat that as the death tick/cause.
    if threshold_candidates:
        min_thr_idx = min(idx for idx, _ in threshold_candidates)
        if idx_nonfinite is None or min_thr_idx <= idx_nonfinite:
            tied = [cause for idx, cause in threshold_candidates if idx == min_thr_idx]
            priority = {c: CAUSE_ORDER.index(c) for c in CAUSE_ORDER}
            best_cause = min(tied, key=lambda c: priority.get(c, 9999))
            return DeathInfo(died=True, death_tick=float(min_thr_idx + 1), cause=best_cause)

    # Otherwise, death is indicated by non-finite child state. Infer cause from last finite row
    # immediately before idx_nonfinite, using "which axis is closest to lethal".
    if idx_nonfinite is None or idx_nonfinite <= 0:
        return DeathInfo(died=True, death_tick=float("nan"), cause="unknown_nonfinite")

    j = idx_nonfinite - 1
    hj = float(h[j]) if np.isfinite(h[j]) else float("nan")
    wj = float(w[j]) if np.isfinite(w[j]) else float("nan")
    ij = float(i[j]) if np.isfinite(i[j]) else float("nan")
    if not (np.isfinite(hj) and np.isfinite(wj) and np.isfinite(ij)):
        return DeathInfo(died=True, death_tick=float(idx_nonfinite + 1), cause="unknown_nonfinite")

    # Score “proximity to lethal” using the same 0–100 scales used in the logs.
    # Warmth: 50 is ideal; death thresholds are 0 (cold) and 100 (heat).
    s_injury = float(np.clip(ij / 100.0, 0.0, 1.0))
    s_hunger = float(np.clip(hj / 100.0, 0.0, 1.0))
    s_cold = float(np.clip((50.0 - wj) / 50.0, 0.0, 1.0))
    s_heat = float(np.clip((wj - 50.0) / 50.0, 0.0, 1.0))

    scores = {"injury": s_injury, "hunger": s_hunger, "cold": s_cold, "heat": s_heat}
    # pick max score; stable tie-break by CAUSE_ORDER
    mx = max(scores.values())
    tied = [k for k, v in scores.items() if v == mx]
    priority = {c: CAUSE_ORDER.index(c) for c in CAUSE_ORDER}
    best = min(tied, key=lambda c: priority.get(c, 9999))
    return DeathInfo(died=True, death_tick=float(idx_nonfinite + 1), cause=best)


def _read_child_arrays(csv_path: str) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, int]:
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None, None, None, 0

    n = int(len(df))
    def _col(name: str) -> np.ndarray | None:
        if name not in df.columns:
            return None
        return pd.to_numeric(df[name], errors="coerce").to_numpy(dtype=float)

    return _col("c0_hunger"), _col("c0_warmth"), _col("c0_injury"), n


def collect_per_run(root: str) -> pd.DataFrame:
    root = os.path.abspath(root)
    # Support multiple layouts:
    # - <root>/<tag>/run_*.csv
    # - <root>/**/run_*.csv
    # - <root>/**/run_seed_*.csv
    paths = []
    paths.extend(glob.glob(os.path.join(root, "*", "run_*.csv")))
    paths.extend(glob.glob(os.path.join(root, "**", "run_*.csv"), recursive=True))
    paths.extend(glob.glob(os.path.join(root, "**", "run_seed_*.csv"), recursive=True))
    paths = sorted(set(paths))
    if not paths:
        raise SystemExit(f"No run logs found under {root} (expected run_*.csv or run_seed_*.csv).")

    rows: list[dict] = []
    skipped = 0
    skipped_no_tag = 0
    for p in paths:
        # Try to infer (plast, init, seed) by scanning parent directory names.
        # Many pipelines embed a tag directory like "E2_random_uniform_seed42".
        parts = os.path.normpath(p).split(os.sep)
        m = None
        for d in reversed(parts[:-1]):  # exclude filename
            mm = RUN_RE.match(d)
            if mm:
                m = mm
                break
        if m is None:
            skipped_no_tag += 1
            continue
        plast = m.group("plast")
        init = m.group("init")
        seed = int(m.group("seed"))

        h, w, inj, nrows = _read_child_arrays(p)
        di = _infer_death_from_arrays(h, w, inj)

        rows.append(
            {
                "path": p,
                "plasticity": plast,
                "init": init,
                "evolve_seed": seed,
                "n_rows": int(nrows),
                "child_died": 1.0 if di.died else 0.0,
                "child_survived": 0.0 if di.died else 1.0,
                "child_death_tick": di.death_tick,
                "death_cause": di.cause,
            }
        )

    out = pd.DataFrame(rows)
    if out.empty:
        raise SystemExit(
            f"No parseable runs under {root}. "
            f"Found {len(paths)} CSVs but none were under a tag dir like 'E2_random_uniform_seed42'."
        )
    print(
        f"[ok] {root}: {len(out)} runs "
        f"(skipped unreadable={skipped}, skipped_no_tag={skipped_no_tag}, total_csv={len(paths)})"
    )
    return out


def aggregate_per_genome(run_df: pd.DataFrame) -> pd.DataFrame:
    keys = ["plasticity", "init", "evolve_seed"]
    g = run_df.groupby(keys, as_index=False)

    # overall death/survival fractions
    base = g.agg(
        n_runs=("path", "count"),
        died_frac=("child_died", "mean"),
        survived_frac=("child_survived", "mean"),
        median_death_tick=("child_death_tick", "median"),
    )

    # per-cause proportions over *all runs* (alive included) + integer counts
    cause_rows: list[pd.DataFrame] = []
    for cause in CAUSE_ORDER:
        mask = run_df["death_cause"].eq(cause)
        tmp = (
            run_df.assign(_is=mask.astype(float))
            .groupby(keys, as_index=False)["_is"]
            .mean()
            .rename(columns={"_is": f"p_{cause}"})
        )
        # count = sum(mask)
        tmp_n = (
            run_df.assign(_is=mask.astype(int))
            .groupby(keys, as_index=False)["_is"]
            .sum()
            .rename(columns={"_is": f"n_{cause}"})
        )
        tmp = tmp.merge(tmp_n, on=keys, how="left")
        cause_rows.append(tmp)

    out = base
    for tmp in cause_rows:
        out = out.merge(tmp, on=keys, how="left")
    out = out.fillna(0.0)

    # stable ordering for plotting
    out["plasticity"] = pd.Categorical(out["plasticity"], categories=PLAST_ORDER, ordered=True)
    out["init"] = pd.Categorical(out["init"], categories=INIT_ORDER, ordered=True)
    out = out.sort_values(["plasticity", "init", "evolve_seed"]).reset_index(drop=True)
    return out


def _stacked_bar(
    ax: plt.Axes,
    df: pd.DataFrame,
    *,
    x_labels: list[str],
    frac_cols: list[str],
    title: str,
) -> None:
    x = np.arange(len(df))
    bottom = np.zeros(len(df), dtype=float)
    for col in frac_cols:
        cause = col.replace("p_", "")
        vals = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy(dtype=float)
        ax.bar(x, vals, bottom=bottom, color=CAUSE_COLORS.get(cause, "#999999"), linewidth=0.0, label=cause)
        bottom += vals
    ax.set_ylim(0.0, 1.0)
    ax.set_title(title, fontsize=11)
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels, rotation=90, fontsize=7)
    ax.set_ylabel("fraction of runs (0–1)")


def plot_by_genome(genome_df: pd.DataFrame, out_png: str) -> None:
    frac_cols = [f"p_{c}" for c in CAUSE_ORDER]

    fig, axes = plt.subplots(3, 3, figsize=(16, 10), sharey=True)
    for i, plast in enumerate(PLAST_ORDER):
        for j, init in enumerate(INIT_ORDER):
            ax = axes[i, j]
            sub = genome_df[(genome_df["plasticity"] == plast) & (genome_df["init"] == init)].copy()
            if sub.empty:
                ax.axis("off")
                continue
            x_labels = [str(int(s)) for s in sub["evolve_seed"].to_numpy()]
            _stacked_bar(
                ax,
                sub,
                x_labels=x_labels,
                frac_cols=frac_cols,
                title=f"{plast} | {init.replace('_', ' ')}",
            )
            if i != 2:
                ax.set_xlabel("")
            else:
                ax.set_xlabel("evolve_seed (genome)")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=min(len(labels), 7), frameon=False, fontsize=9)
    fig.suptitle(
        "Child outcome by genome (100% stacked across runs)\n"
        "Interpretation: 1.0 means 100% of runs for that genome ended the same way.",
        y=0.98,
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_by_init(run_df: pd.DataFrame, out_png: str) -> None:
    keys = ["plasticity", "init"]
    # cause proportions across all runs within each (plast, init)
    acc = run_df.groupby(keys, as_index=False).agg(n_runs=("path", "count"))
    for cause in CAUSE_ORDER:
        col = f"p_{cause}"
        coln = f"n_{cause}"
        acc[col] = (
            run_df.assign(_is=(run_df["death_cause"].eq(cause)).astype(float))
            .groupby(keys)["_is"]
            .mean()
            .to_numpy()
        )
        acc[coln] = (
            run_df.assign(_is=(run_df["death_cause"].eq(cause)).astype(int))
            .groupby(keys)["_is"]
            .sum()
            .to_numpy()
        )

    acc["plasticity"] = pd.Categorical(acc["plasticity"], categories=PLAST_ORDER, ordered=True)
    acc["init"] = pd.Categorical(acc["init"], categories=INIT_ORDER, ordered=True)
    acc = acc.sort_values(["plasticity", "init"]).reset_index(drop=True)

    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.8), sharey=True)
    frac_cols = [f"p_{c}" for c in CAUSE_ORDER]
    for i, plast in enumerate(PLAST_ORDER):
        ax = axes[i]
        sub = acc[acc["plasticity"] == plast].copy()
        if sub.empty:
            ax.axis("off")
            continue
        x_labels = [s.replace("_", " ") for s in sub["init"].astype(str).to_list()]
        _stacked_bar(ax, sub, x_labels=x_labels, frac_cols=frac_cols, title=f"{plast} (n={int(sub['n_runs'].sum())} runs)")
        ax.set_xlabel("init")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=min(len(labels), 7), frameon=False, fontsize=9)
    fig.suptitle(
        "Child outcome mix by init (100% stacked across runs)\n"
        "Interpretation: 1.0 means 100% of runs ended that way.",
        y=0.98,
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Root directory containing <tag>/run_*.csv (tag like E2_random_uniform_seed42).")
    ap.add_argument("--out-dir", required=True, help="Output directory for PNGs/CSVs.")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    run_df = collect_per_run(args.root)
    genome_df = aggregate_per_genome(run_df)

    run_csv = os.path.join(args.out_dir, "child_death_causes_per_run.csv")
    genome_csv = os.path.join(args.out_dir, "child_death_causes_per_genome.csv")
    run_df.to_csv(run_csv, index=False)
    genome_df.to_csv(genome_csv, index=False)

    # Convenience summaries with counts only (easy to read in a spreadsheet)
    counts_cols = ["plasticity", "init", "evolve_seed", "n_runs"] + [f"n_{c}" for c in CAUSE_ORDER]
    frac_cols = ["plasticity", "init", "evolve_seed", "n_runs"] + [f"p_{c}" for c in CAUSE_ORDER]
    genome_df[counts_cols].to_csv(os.path.join(args.out_dir, "death_cause_counts_by_genome.csv"), index=False)
    genome_df[frac_cols].to_csv(os.path.join(args.out_dir, "death_cause_fracs_by_genome.csv"), index=False)

    # By-init counts
    by_init = (
        run_df.groupby(["plasticity", "init", "death_cause"], as_index=False)
        .agg(n=("path", "count"))
    )
    by_init.to_csv(os.path.join(args.out_dir, "death_cause_counts_by_init.csv"), index=False)

    plot_by_genome(genome_df, os.path.join(args.out_dir, "death_survival_stacked_by_genome.png"))
    plot_by_init(run_df, os.path.join(args.out_dir, "death_cause_mix_by_init.png"))

    print(f"[ok] wrote:\n  - {run_csv}\n  - {genome_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

