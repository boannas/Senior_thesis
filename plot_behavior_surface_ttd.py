#!/usr/bin/env python3
"""
plot_behavior_surface_ttd.py
============================
Make **3D interpolated smooth surfaces** of child outcome as a function of two
behavioral conditional-probability axes:

  x = P(motivation | state) metric  (from conditional_metrics_single.csv)
  y = P(motivation | state) metric
  z = child_ttd_norm

Data source
-----------
Each condition root should have been processed by:
  python3 plot_runlog_conditional_behavior.py --root <root> --extended

We read:
  <root>/<metrics-subdir>/conditional_metrics_single.csv

Then aggregate **per-genome** means (mean over rollout seeds) for each:
  (condition, plasticity, init, evolve_seed)

Plotting
--------
Because genomes are scattered in (x,y), we interpolate a smooth surface using
matplotlib triangulation (no scipy):
  matplotlib.tri.LinearTriInterpolator

We mask outside the convex hull (the interpolator returns masked arrays).

Outputs
-------
Under --out-dir:
  surface_<cond>__<init>__<plast>.png   (one file per condition × init; 3 panels)
  surface_points_<cond>__<init>.csv     (the per-genome points used)

Example
-------
python3 plot_behavior_surface_ttd.py \\
  --cond base=FinalGenomeRollouts_normal_seen \\
  --cond unseen_hard=FinalGenomeRollouts_unseen_hard \\
  --x P_sel_Care_given_child_hungry \\
  --y P_sel_Forage_given_child_hungry \\
  --init pro_maternal \\
  --out-dir paper_figures_surface
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.tri as mtri
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]
PLAST_TITLES = {"E2": "E2 (no plasticity)", "P1": "P1", "P2": "P2"}

plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "axes.grid": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)


def _parse_kv(spec: str) -> tuple[str, list[str]]:
    if "=" not in spec:
        raise SystemExit(f"Expected label=path, got: {spec!r}")
    k, v = spec.split("=", 1)
    roots = []
    for part in v.split(","):
        part = part.strip()
        if part:
            roots.append(os.path.abspath(part))
    if not roots:
        raise SystemExit(f"Expected non-empty path(s) after '=', got: {spec!r}")
    return k.strip(), roots


def _load_points_for_condition(
    condition: str,
    roots: list[str],
    *,
    metrics_subdir: str,
    x_col: str,
    y_col: str,
) -> "pd.DataFrame":
    dfs = []
    for root in roots:
        csv_path = os.path.join(root, metrics_subdir, "conditional_metrics_single.csv")
        if not os.path.isfile(csv_path):
            print(f"[warn] missing metrics CSV for {condition}: {csv_path}", file=sys.stderr)
            continue
        df = pd.read_csv(csv_path)
        df["condition"] = condition
        dfs.append(df)
    if not dfs:
        return pd.DataFrame()
    df_all = pd.concat(dfs, ignore_index=True)

    need = {"plasticity", "init", "evolve_seed", "child_ttd_norm", x_col, y_col}
    missing = need - set(df_all.columns)
    if missing:
        print(f"[warn] {condition} missing columns {missing} in metrics CSV(s)", file=sys.stderr)
        return pd.DataFrame()

    keep = ["condition", "plasticity", "init", "evolve_seed", "child_ttd_norm", x_col, y_col]
    df_all = df_all[keep].copy()
    for c in ["child_ttd_norm", x_col, y_col]:
        df_all[c] = pd.to_numeric(df_all[c], errors="coerce")

    # Per-genome mean across rollout seeds.
    pg = (
        df_all.groupby(["condition", "plasticity", "init", "evolve_seed"], as_index=False)
        .mean(numeric_only=True)
        .dropna(subset=["child_ttd_norm", x_col, y_col])
    )
    return pg


def _surface_interpolate(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    *,
    grid_n: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns (Xg, Yg, Zg_masked) for plotting with plot_surface.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    z = np.asarray(z, dtype=float)
    m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[m], y[m], z[m]
    if x.size < 6:
        # too few points for triangulation
        Xg = Yg = np.full((grid_n, grid_n), np.nan, dtype=float)
        Zg = np.ma.masked_all((grid_n, grid_n), dtype=float)
        return Xg, Yg, Zg

    tri = mtri.Triangulation(x, y)
    itp = mtri.LinearTriInterpolator(tri, z)

    xi = np.linspace(float(np.nanmin(x)), float(np.nanmax(x)), grid_n)
    yi = np.linspace(float(np.nanmin(y)), float(np.nanmax(y)), grid_n)
    Xg, Yg = np.meshgrid(xi, yi)
    Zg = itp(Xg, Yg)  # masked array outside hull
    return Xg, Yg, Zg


def _plot_condition_init(
    pg_cond: "pd.DataFrame",
    *,
    condition: str,
    init: str,
    x_col: str,
    y_col: str,
    out_dir: str,
    grid_n: int,
) -> None:
    sub_init = pg_cond[pg_cond["init"] == init].copy()
    if sub_init.empty:
        print(f"[warn] no points for condition={condition} init={init}", file=sys.stderr)
        return

    # Save points for reproducibility.
    pts_path = os.path.join(out_dir, f"surface_points_{condition}__{init}.csv")
    sub_init.to_csv(pts_path, index=False)

    fig = plt.figure(figsize=(13.8, 4.4))
    axes = []
    for i, plast in enumerate(PLAST_ORDER):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        axes.append(ax)
        sub = sub_init[sub_init["plasticity"] == plast]
        if sub.empty:
            ax.set_title(PLAST_TITLES.get(plast, plast))
            continue

        x = sub[x_col].to_numpy(dtype=float)
        y = sub[y_col].to_numpy(dtype=float)
        z = sub["child_ttd_norm"].to_numpy(dtype=float)

        Xg, Yg, Zg = _surface_interpolate(x, y, z, grid_n=grid_n)
        # Surface
        ax.plot_surface(
            Xg,
            Yg,
            Zg,
            cmap="viridis",
            linewidth=0,
            antialiased=True,
            alpha=0.88,
        )
        # Also show the raw points lightly for sanity.
        ax.scatter(x, y, z, s=10, c="k", alpha=0.18, depthshade=False)

        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xlabel(x_col)
        ax.set_ylabel(y_col)
        ax.set_zlabel("child_ttd_norm")

        # View angle tuned for readability.
        ax.view_init(elev=26, azim=-135)

    fig.suptitle(
        f"{condition} — init={init.replace('_',' ')}\nInterpolated surface: child_ttd_norm vs ({x_col}, {y_col})",
        y=1.03,
        fontsize=12,
    )
    fig.tight_layout()
    out_png = os.path.join(out_dir, f"surface_{condition}__{init}.png")
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cond", action="append", required=True, help="Repeatable: label=/path/to/root1,/path/to/root2,...")
    ap.add_argument("--x", required=True, help="Column for x-axis (e.g. P_sel_Care_given_child_hungry).")
    ap.add_argument("--y", required=True, help="Column for y-axis (e.g. P_sel_Forage_given_child_hungry).")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--init", default="all", help="One of anti_maternal/random_uniform/pro_maternal/all (default all).")
    ap.add_argument("--metrics-subdir", default="figures_conditional_extended", help="Subdir containing conditional_metrics_single.csv")
    ap.add_argument("--grid-n", type=int, default=35, help="Interpolation grid resolution (default 35).")
    ap.add_argument(
        "--condition-order",
        default="",
        help="Optional comma-separated order of condition labels to process (default: input order).",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    cond_roots: dict[str, list[str]] = {}
    input_order: list[str] = []
    for raw in args.cond:
        label, roots = _parse_kv(raw)
        if label not in cond_roots:
            cond_roots[label] = []
            input_order.append(label)
        cond_roots[label].extend(roots)

    if args.condition_order.strip():
        cond_order = [c.strip() for c in args.condition_order.split(",") if c.strip()]
        for c in input_order:
            if c not in cond_order:
                cond_order.append(c)
    else:
        cond_order = input_order

    if args.init != "all" and args.init not in INIT_ORDER:
        raise SystemExit(f"--init must be one of {INIT_ORDER + ['all']}, got: {args.init!r}")
    inits = INIT_ORDER if args.init == "all" else [args.init]

    for cond in cond_order:
        roots = cond_roots.get(cond, [])
        if not roots:
            continue
        pg = _load_points_for_condition(cond, roots, metrics_subdir=args.metrics_subdir, x_col=args.x, y_col=args.y)
        if pg.empty:
            continue
        for init in inits:
            _plot_condition_init(
                pg,
                condition=cond,
                init=init,
                x_col=args.x,
                y_col=args.y,
                out_dir=out_dir,
                grid_n=int(args.grid_n),
            )

    idx = os.path.join(out_dir, "surface_index.txt")
    with open(idx, "w", encoding="utf-8") as f:
        f.write("Behavior surface outputs:\n")
        for name in sorted(os.listdir(out_dir)):
            if name.endswith((".png", ".csv", ".txt")):
                f.write(f"  {name}\n")
    print("[ok] wrote", idx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

