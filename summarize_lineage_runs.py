"""
summarize_lineage_runs.py

Walk a Baldwin/E2-E3 results tree, parse every `lineage_generations.csv`,
and emit a tidy one-row-per-run summary CSV with the metrics from the
analysis plan (final_fitness, AUC, gen_to_90, drift, etc.).

Expected layout under --root:

    <root>/Evolved_results/{easy,normal,hard}/<run_dir>/lineage_generations.csv
    <root>/Plastic_1_results/{easy,normal,hard}/<run_dir>/lineage_generations.csv
    <root>/Plastic_2_results/{easy,normal,hard}/<run_dir>/lineage_generations.csv

    Sweep (RUN_GUIDE): ``<root>/Sweep_results/{easy,normal,hard}/E2_thr*_g*_i*_a*_seed*/`` (same for P1_/P2_).

Run-dir name is parsed for: thrT_gG_iI_a{alpha}_seed{S}, e.g.
  E2_thr0_g10_i10_a0p5_seed42
  E3_outadapt_glob_pertick_thr2_g20_i10_a0p3_seed42
  E3_outcome_loc_pertick_thr1_g15_i20_a0p7_seed42

Usage:
  python summarize_lineage_runs.py
  python summarize_lineage_runs.py --root Baldwin_Experiment --out Baldwin_Experiment/lineage_summary.csv
  python summarize_lineage_runs.py --final-window 300 --init-window 200 --ma-window 80
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from glob import glob

import numpy as np
import pandas as pd

# ---------- mapping from on-disk folder names to short tags ----------
PLASTIC_TAG = {
    "Evolved_results": "E2",
    "Plastic_1_results": "P1",
    "Plastic_2_results": "P2",
}
WORLD_TAG = {"easy", "normal", "hard"}

RUN_NAME_RE = re.compile(
    r"thr(?P<thr>\d+)_g(?P<g>\d+)_i(?P<i>\d+)_a(?P<alpha>[0-9]+p[0-9]+)_seed(?P<seed>\d+)"
)


def _parse_alpha_str(s: str) -> float:
    return float(s.replace("p", "."))


def _detect_plasticity_and_world(csv_path: str, root: str) -> tuple[str | None, str | None]:
    """Pull the plasticity tag (E2/P1/P2) and world tag (easy/normal/hard) from path parts.

    Also supports RUN_GUIDE sweep layout: ``Sweep_results/<world>/<E2|P1|P2>_thr*_.../`` where
    plasticity is taken from the run folder prefix when ``Evolved_results/`` is absent.
    """
    rel = os.path.relpath(csv_path, root)
    parts = rel.split(os.sep)
    plasticity: str | None = None
    world: str | None = None
    for p in parts:
        if p in PLASTIC_TAG and plasticity is None:
            plasticity = PLASTIC_TAG[p]
        if p in WORLD_TAG and world is None:
            world = p
    if plasticity is None:
        run_dir = os.path.basename(os.path.dirname(csv_path))
        if run_dir.startswith("E2_"):
            plasticity = "E2"
        elif run_dir.startswith("P1_"):
            plasticity = "P1"
        elif run_dir.startswith("P2_"):
            plasticity = "P2"
    return plasticity, world


def _per_run_metrics(
    df: pd.DataFrame,
    *,
    ma_window: int,
    init_window: int,
    final_window: int,
) -> dict:
    """Distill one lineage CSV trajectory into a dict of summary metrics."""
    df = df.sort_values("generation").reset_index(drop=True)
    n = len(df)
    if n == 0:
        return {}

    init_w = max(1, min(init_window, max(1, n // 5)))
    final_w = max(1, min(final_window, max(1, n // 5)))
    ma_w = max(1, min(ma_window, max(1, n // 20)))

    fit = pd.to_numeric(df["fitness"], errors="coerce")
    ma = fit.rolling(window=ma_w, min_periods=1).mean()

    final_fitness = float(ma.iloc[-final_w:].mean())
    init_fitness = float(ma.iloc[:init_w].mean())

    # AUC under the smoothed fitness curve, normalized by the generation span,
    # so it is comparable across runs of different length (= mean fitness over time).
    last_gen = int(df["generation"].iloc[-1]) if n > 1 else 1
    if last_gen <= 0:
        last_gen = max(1, n - 1)
    auc_fitness = float(np.trapz(ma.values, df["generation"].values) / max(1, last_gen))

    # gen_to_90 = first generation where smoothed fitness reaches 0.9 * final_fitness
    target = 0.9 * final_fitness
    hits = ma >= target
    if hits.any():
        gen_to_90 = int(df.loc[hits.idxmax(), "generation"])
    else:
        gen_to_90 = int(df["generation"].iloc[-1])

    # u_drift trajectory: high early, low late = genetic assimilation
    if "mean_u_drift_end" in df.columns:
        ud = pd.to_numeric(df["mean_u_drift_end"], errors="coerce")
        init_drift = float(ud.iloc[:init_w].mean())
        final_drift = float(ud.iloc[-final_w:].mean())
        drift_drop = init_drift - final_drift
    else:
        init_drift = final_drift = drift_drop = float("nan")

    # acceptance statistics
    if "accepted" in df.columns:
        accepted = (
            df["accepted"].astype(str).str.strip().str.lower().isin({"true", "1"})
        )
        accept_rate = float(accepted.mean())
        n_accepted = int(accepted.sum())
    else:
        accept_rate = float("nan")
        n_accepted = -1

    def _last_mean(col: str) -> float:
        if col not in df.columns:
            return float("nan")
        s = pd.to_numeric(df[col], errors="coerce")
        return float(s.iloc[-final_w:].mean())

    return {
        "num_generations": int(n),
        "final_fitness": final_fitness,
        "init_fitness": init_fitness,
        "fitness_improvement": final_fitness - init_fitness,
        "auc_fitness": auc_fitness,
        "gen_to_90": gen_to_90,
        "init_drift": init_drift,
        "final_drift": final_drift,
        "drift_drop": drift_drop,
        "n_accepted": n_accepted,
        "accept_rate": accept_rate,
        "final_child_survival": _last_mean("mean_child_survival"),
        "final_child_ttd_norm": _last_mean("mean_child_ttd_norm"),
        "final_mother_ttd_norm": _last_mean("mean_mother_ttd_norm"),
        "final_child_injury": _last_mean("mean_child_injury"),
        "final_overall_def_episode": _last_mean("mean_overall_deficit_episode"),
        "final_overall_def_end": _last_mean("mean_overall_deficit_end"),
        "final_local_def_episode": _last_mean("mean_local_deficit_episode"),
        "final_local_def_end": _last_mean("mean_local_deficit_end"),
    }


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--root",
        default="Baldwin_Experiment",
        help="Root dir that contains Evolved_results/, Plastic_1_results/, Plastic_2_results/.",
    )
    p.add_argument(
        "--out",
        default=None,
        help="Output CSV path (default: <root>/lineage_summary.csv).",
    )
    p.add_argument("--ma-window", type=int, default=50, help="Rolling window for fitness smoothing.")
    p.add_argument("--init-window", type=int, default=100, help="# of early generations averaged.")
    p.add_argument(
        "--final-window",
        type=int,
        default=200,
        help="# of late generations averaged for 'final_*' metrics.",
    )
    args = p.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] root not found: {root}", file=sys.stderr)
        return 2
    out_path = os.path.abspath(args.out) if args.out else os.path.join(root, "lineage_summary.csv")

    csvs = sorted(glob(os.path.join(root, "**", "lineage_generations.csv"), recursive=True))
    if not csvs:
        print(f"[error] no lineage_generations.csv found under {root}", file=sys.stderr)
        return 2

    rows: list[dict] = []
    skipped: list[tuple[str, str]] = []
    for csv_path in csvs:
        plasticity, world = _detect_plasticity_and_world(csv_path, root)
        run_dir = os.path.basename(os.path.dirname(csv_path))
        m = RUN_NAME_RE.search(run_dir)
        if plasticity is None:
            skipped.append((csv_path, "no plasticity tag in path"))
            continue
        if world is None:
            skipped.append((csv_path, "no world tag in path"))
            continue
        if m is None:
            skipped.append((csv_path, "run dir name did not match thr/g/i/a/seed pattern"))
            continue
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            skipped.append((csv_path, f"read failed: {e!r}"))
            continue
        if "fitness" not in df.columns or "generation" not in df.columns:
            skipped.append((csv_path, "missing fitness/generation columns"))
            continue

        metrics = _per_run_metrics(
            df,
            ma_window=args.ma_window,
            init_window=args.init_window,
            final_window=args.final_window,
        )
        if not metrics:
            skipped.append((csv_path, "empty CSV after sort"))
            continue

        rows.append(
            {
                "plasticity": plasticity,
                "world": world,
                "threats": int(m.group("thr")),
                "grid": int(m.group("g")),
                "food_interval": int(m.group("i")),
                "alpha_child": _parse_alpha_str(m.group("alpha")),
                "seed": int(m.group("seed")),
                "run_dir": run_dir,
                "csv_path": os.path.relpath(csv_path, root),
                **metrics,
            }
        )

    if not rows:
        print("[error] no valid runs parsed.", file=sys.stderr)
        for path, why in skipped:
            print(f"  skipped: {path} -- {why}", file=sys.stderr)
        return 2

    out_df = pd.DataFrame(rows)
    front = ["plasticity", "world", "threats", "grid", "food_interval", "alpha_child", "seed"]
    cols = front + [c for c in out_df.columns if c not in front]
    out_df = out_df[cols].sort_values(front).reset_index(drop=True)

    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    out_df.to_csv(out_path, index=False)

    print(f"[ok] wrote {out_path}")
    print(f"     rows={len(out_df)}  skipped={len(skipped)}")
    if skipped:
        print("     (use --help and check folder layout if any of those should have been parsed)")
        for path, why in skipped[:5]:
            print(f"     - {path}  ({why})")
        if len(skipped) > 5:
            print(f"     ... and {len(skipped) - 5} more")

    # quick groupby preview to stdout
    try:
        preview = (
            out_df.groupby(["plasticity", "world", "alpha_child"])
            .agg(
                n=("final_fitness", "size"),
                mean_final_fit=("final_fitness", "mean"),
                mean_drift_drop=("drift_drop", "mean"),
            )
            .round(4)
            .reset_index()
        )
        print("\n[preview] mean final_fitness and drift_drop by (plasticity, world, alpha):")
        print(preview.to_string(index=False))
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
