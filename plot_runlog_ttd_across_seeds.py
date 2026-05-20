"""
Plot mother/child time-to-death (TTD) across many headless run logs.

This reads RunLogger-style CSVs produced by `headless_rollout_log.py --out-dir ...`,
e.g. `run_1001.csv`, `run_1002.csv`, ...

Death tick convention (matches plot_logged_run_aggregate.py):
- Mother "dies" when `m0_energy` becomes non-finite (NaN) in the log.
- Child "dies" when `c0_hunger` becomes non-finite (NaN) in the log.
- If a column never becomes non-finite, the run is treated as right-censored at
  `max_tick_observed` (last tick in that CSV).

Outputs:
- `ttd_across_seeds.png` (boxplots + jitter + summary text)
- `ttd_across_seeds.csv` (seed, mother_death_tick, child_death_tick, days)

Usage:
  python plot_runlog_ttd_across_seeds.py --runlog-glob "result_experiment/run_log_1001_1128/run_*.csv"
  python plot_runlog_ttd_across_seeds.py --runlog-dir result_experiment/run_log_1001_1128
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    print("Requires pandas and matplotlib:", e, file=sys.stderr)
    sys.exit(1)


def _first_nonfinite_tick(series: np.ndarray) -> int | None:
    """Return first index where value is non-finite; None if never."""
    for i, v in enumerate(series):
        if not np.isfinite(v):
            return int(i + 1)  # ticks in logs are 1-based in the CSV
    return None


def _seed_from_path(path: str) -> int | None:
    m = re.search(r"run_(\d+)\.csv$", os.path.basename(path))
    return int(m.group(1)) if m else None


def _load_death_ticks(csv_path: str) -> dict:
    df = pd.read_csv(csv_path)
    if "tick" not in df.columns:
        raise ValueError(f"Missing 'tick' column: {csv_path}")
    max_tick = int(np.nanmax(pd.to_numeric(df["tick"], errors="coerce")))

    def _col_tick(col: str) -> int:
        if col not in df.columns:
            raise ValueError(f"Missing column {col!r} in {csv_path}")
        s = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        t = _first_nonfinite_tick(s)
        return int(t) if t is not None else int(max_tick)

    return {
        "seed": _seed_from_path(csv_path),
        "path": csv_path,
        "max_tick": int(max_tick),
        "mother_death_tick": _col_tick("m0_energy"),
        "child_death_tick": _col_tick("c0_hunger"),
    }


def main() -> None:
    p = argparse.ArgumentParser(description="Plot TTD across headless run logs.")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--runlog-dir", type=str, help="Directory containing run_*.csv")
    g.add_argument("--runlog-glob", type=str, help="Glob for run logs (e.g. path/run_*.csv)")
    p.add_argument("--day-step", type=int, default=100, help="Ticks per day (for y-axis in days).")
    p.add_argument("--out", "-o", type=str, default=None, help="Output PNG path (default: <dir>/ttd_across_seeds.png)")
    p.add_argument("--out-csv", type=str, default=None, help="Output summary CSV (default: <dir>/ttd_across_seeds.csv)")
    p.add_argument("--title", type=str, default="", help="Optional title prefix.")
    args = p.parse_args()

    if args.runlog_dir:
        runlog_dir = os.path.abspath(args.runlog_dir)
        paths = sorted(glob.glob(os.path.join(runlog_dir, "run_*.csv")))
    else:
        paths = sorted(glob.glob(args.runlog_glob))
        runlog_dir = os.path.abspath(os.path.dirname(paths[0])) if paths else os.getcwd()

    if not paths:
        raise SystemExit("No run logs found.")

    rows = []
    for path in paths:
        try:
            rows.append(_load_death_ticks(path))
        except Exception as e:
            print(f"[skip] {path}: {e}", file=sys.stderr)

    if not rows:
        raise SystemExit("No usable run logs (missing required columns).")

    df = pd.DataFrame(rows)
    day_step = float(args.day_step)
    df["mother_days"] = df["mother_death_tick"] / day_step
    df["child_days"] = df["child_death_tick"] / day_step

    # Output CSV
    out_csv = args.out_csv or os.path.join(runlog_dir, "ttd_across_seeds.csv")
    df.sort_values(["seed"], na_position="last").to_csv(out_csv, index=False)

    # Plot
    out_png = args.out or os.path.join(runlog_dir, "ttd_across_seeds.png")
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))

    def _panel(ax, values: np.ndarray, label: str, color: str):
        values = values[np.isfinite(values)]
        # matplotlib 3.9+: labels renamed to tick_labels
        ax.boxplot([values], labels=[label], showfliers=True, patch_artist=True)
        # jittered points
        x = np.ones_like(values)
        jitter = (np.random.RandomState(0).randn(len(values)) * 0.03)
        ax.scatter(x + jitter, values, s=14, alpha=0.35, color="black", edgecolors="none", rasterized=True)
        ax.set_ylabel("Days")
        ax.grid(True, axis="y", alpha=0.3)
        # summary text
        q1, med, q3 = np.nanpercentile(values, [25, 50, 75])
        mean = float(np.nanmean(values))
        ax.set_title(f"{label} TTD (n={len(values)})\\nmedian={med:.2f}d  IQR=[{q1:.2f},{q3:.2f}]  mean={mean:.2f}d")
        for patch in ax.artists:
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

    _panel(axes[0], df["child_days"].to_numpy(dtype=float), "Child", color="#ffb74d")
    _panel(axes[1], df["mother_days"].to_numpy(dtype=float), "Mother", color="#90caf9")

    if args.title:
        fig.suptitle(args.title, y=1.02, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    plt.close(fig)

    print("Wrote", out_png)
    print("Wrote", out_csv)


if __name__ == "__main__":
    main()

