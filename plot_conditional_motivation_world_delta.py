#!/usr/bin/env python3
"""
plot_conditional_motivation_world_delta.py
===========================================
Compare **extended** conditional motivation probabilities between a **base**
rollout world (e.g. training / normal_seen) and one or more **unseen** roots,
**genome-matched** on (plasticity, init, evolve_seed).

For each ``P_sel_{M}_given_{mask}`` column we form::

    Δ = P_unseen(M | mask) - P_base(M | mask)

per genome (mean within seed over run CSVs, then difference). Summaries use
**median Δ** across genomes (and optional IQR for diagnostics).

Why this is easier to read than multi-world stack plots
--------------------------------------------------------
- Absolute stacks for base vs unseen on the same figure double the visual load.
- **Δ heatmaps** put the story on one diverging scale: “unseen shifts time
  toward Care or away from Forage under this mask?”
- Pairing genomes removes confounds from different evolved populations.

Outputs (per unseen tag, under ``--out-dir``)
---------------------------------------------
  - ``delta_median_heatmap__<tag>.png`` — 3×3 panels (plasticity × init),
    each cell is 4 motivations × 6 state masks, color = median Δ
  - ``delta_per_genome__<tag>.csv`` — long-ish table of per-genome deltas
  - ``delta_median_table__<tag>.csv`` — wide table of median Δ per (plast, init)

Inputs
------
Either ``--base-csv`` / ``--unseen <tag>=<csv>`` **or** ``--base-root`` /
``--unseen <tag>=<root>`` where *root* contains
``figures_conditional_extended/conditional_metrics_single.csv`` (from
``plot_runlog_conditional_behavior.py --extended``).

Example::

  python3 plot_conditional_motivation_world_delta.py \\
      --base-root FinalGenomeRollouts_normal_seen \\
      --unseen unseen_hard=FinalGenomeRollouts_unseen_hard \\
      --unseen unseen_food30=FinalGenomeRollouts_unseen_food30 \\
      --out-dir paper_figures_conditional_delta
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

import plot_runlog_conditional_behavior as crb

plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

CONDITION_KEYS = [
    "child_hungry",
    "child_cold",
    "child_injured",
    "mother_fear_hi",
    "mother_energy_lo",
    "mother_stress_hi",
]
CONDITION_XLABELS = ["Hungry", "Cold", "Injured", "Fear↑", "E low", "Stress↑"]


def _psel_columns() -> list[str]:
    cols = []
    for ck in CONDITION_KEYS:
        for mot in crb.MOTIVATIONS:
            cols.append(f"P_sel_{mot}_given_{ck}")
    return cols


def _resolve_cond_csv(path: str) -> str:
    path = os.path.abspath(path)
    if path.endswith(".csv") and os.path.isfile(path):
        return path
    cand = os.path.join(path, "figures_conditional_extended", "conditional_metrics_single.csv")
    if os.path.isfile(cand):
        return cand
    raise SystemExit(f"Could not find extended conditional CSV for {path!r} (tried file and {cand})")


def _load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    need = {"plasticity", "init", "evolve_seed"}
    if not need.issubset(df.columns):
        raise SystemExit(f"{path} missing columns {need}")
    for c in _psel_columns():
        if c not in df.columns:
            raise SystemExit(f"{path} missing extended column {c!r} (re-run with --extended)")
    return df


def _per_seed_means(df: pd.DataFrame) -> pd.DataFrame:
    pcols = _psel_columns()
    for c in pcols:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[pcols].mean()


def _merge_delta(base_ps: pd.DataFrame, oth_ps: pd.DataFrame, *, suffix_other: str) -> pd.DataFrame:
    pcols = _psel_columns()
    m = base_ps.merge(oth_ps, on=["plasticity", "init", "evolve_seed"], how="inner", suffixes=("_b", "_u"))
    if m.empty:
        raise SystemExit("No overlapping genomes after inner join (check seeds / roots).")
    for c in pcols:
        m[f"d_{c}"] = pd.to_numeric(m[f"{c}_u"], errors="coerce") - pd.to_numeric(m[f"{c}_b"], errors="coerce")
    return m


def _median_matrix(sub: pd.DataFrame) -> np.ndarray:
    """4 x 6 median delta for one (plasticity, init) slice."""
    pcols = _psel_columns()
    mat = np.full((len(crb.MOTIVATIONS), len(CONDITION_KEYS)), np.nan, dtype=float)
    if sub.empty:
        return mat
    for mi, mot in enumerate(crb.MOTIVATIONS):
        for ci, ck in enumerate(CONDITION_KEYS):
            col = f"d_P_sel_{mot}_given_{ck}"
            xs = pd.to_numeric(sub[col], errors="coerce").dropna().to_numpy(dtype=float)
            mat[mi, ci] = float(np.median(xs)) if xs.size else float("nan")
    return mat


def _plot_delta_heatmaps(
    merged: pd.DataFrame,
    *,
    unseen_tag: str,
    base_label: str,
    out_png: str,
    vmax: float | None,
) -> None:
    dcols = [c for c in merged.columns if c.startswith("d_P_sel_")]
    # symmetric color limit from data
    finite = merged[dcols].to_numpy(dtype=float)
    finite = finite[np.isfinite(finite)]
    lim = float(np.nanmax(np.abs(finite))) if finite.size else 0.1
    if not np.isfinite(lim) or lim <= 0:
        lim = 0.1
    if vmax is not None:
        lim = min(lim, vmax)

    fig, axes = plt.subplots(3, 3, figsize=(12.5, 10.0), layout="constrained")
    ims = []
    for i, plast in enumerate(crb.PLAST_ORDER):
        for j, init in enumerate(crb.INIT_ORDER):
            ax = axes[i, j]
            ax.grid(False)
            sub = merged[(merged["plasticity"] == plast) & (merged["init"] == init)]
            mat = _median_matrix(sub)
            im = ax.imshow(mat, aspect="auto", cmap="RdBu_r", vmin=-lim, vmax=lim)
            ims.append(im)
            ax.set_xticks(np.arange(len(CONDITION_KEYS)))
            if i == 2:
                ax.set_xticklabels(CONDITION_XLABELS, rotation=35, ha="right", fontsize=7)
            else:
                ax.set_xticklabels([])
            ax.set_yticks(np.arange(len(crb.MOTIVATIONS)))
            ax.set_yticklabels(list(crb.MOTIVATIONS) if j == 0 else [])
            ax.set_title(f"{plast} | {init.replace('_', ' ')}", fontsize=8)
    for j in range(3):
        axes[2, j].set_xlabel("state mask (conditioning event)")

    fig.colorbar(ims[-1], ax=axes, shrink=0.55, label="median ΔP(sel | mask)\n(unseen − base)")
    n_pairs = len(merged)
    fig.suptitle(
        f"{unseen_tag}: ΔP(sel|mask) = unseen − base\n"
        f"base: {base_label}  |  paired rows: {n_pairs} (plasticity×init×genome present in both worlds)\n"
        "rows = motivation, columns = state mask; cell = median Δ across genomes",
        fontsize=10,
    )
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _write_tables(merged: pd.DataFrame, unseen_tag: str, out_dir: str) -> None:
    dcols = [c for c in merged.columns if c.startswith("d_P_sel_")]
    merged[dcols + ["plasticity", "init", "evolve_seed"]].to_csv(
        os.path.join(out_dir, f"delta_per_genome__{unseen_tag}.csv"), index=False
    )
    rows = []
    for (plast, init), sub in merged.groupby(["plasticity", "init"], dropna=False):
        rec = {"plasticity": plast, "init": init, "n_genomes": int(sub["evolve_seed"].nunique())}
        for dc in dcols:
            xs = pd.to_numeric(sub[dc], errors="coerce").dropna().to_numpy(dtype=float)
            rec[f"median__{dc}"] = float(np.median(xs)) if xs.size else float("nan")
            rec[f"iqr__{dc}"] = (
                float(np.subtract(*np.percentile(xs, [75, 25]))) if xs.size >= 2 else float("nan")
            )
        rows.append(rec)
    pd.DataFrame(rows).to_csv(os.path.join(out_dir, f"delta_median_table__{unseen_tag}.csv"), index=False)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-csv", default=None, help="Extended conditional_metrics_single.csv for base world.")
    ap.add_argument("--base-root", default=None, help="Rollout root; uses .../figures_conditional_extended/...")
    ap.add_argument(
        "--unseen",
        action="append",
        default=[],
        help="Repeatable: tag=path (CSV, rollout root, or path to extended folder).",
    )
    ap.add_argument("--out-dir", required=True, help="Output directory.")
    ap.add_argument("--vmax", type=float, default=None, help="Cap diverging colormap at this |Δ| (else auto).")
    args = ap.parse_args()

    if not args.base_csv and not args.base_root:
        print("[error] need --base-csv or --base-root", file=sys.stderr)
        return 2
    if not args.unseen:
        print("[error] need at least one --unseen tag=path", file=sys.stderr)
        return 2

    base_path = args.base_csv or _resolve_cond_csv(args.base_root)
    base_label = os.path.basename(os.path.dirname(base_path)) or base_path
    base_ps = _per_seed_means(_load_csv(base_path))

    os.makedirs(args.out_dir, exist_ok=True)

    for spec in args.unseen:
        if "=" not in spec:
            print(f"[error] bad --unseen (want tag=path): {spec}", file=sys.stderr)
            return 2
        tag, raw = spec.split("=", 1)
        tag = tag.strip()
        raw = raw.strip()
        oth_path = _resolve_cond_csv(raw)
        oth_ps = _per_seed_means(_load_csv(oth_path))
        merged = _merge_delta(base_ps, oth_ps, suffix_other=tag)
        print(f"[ok] {tag}: paired genomes = {len(merged)} rows (unique seeds {merged['evolve_seed'].nunique()})")

        _write_tables(merged, tag, args.out_dir)
        _plot_delta_heatmaps(
            merged,
            unseen_tag=tag,
            base_label=base_label,
            out_png=os.path.join(args.out_dir, f"delta_median_heatmap__{tag}.png"),
            vmax=args.vmax,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
