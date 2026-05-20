#!/usr/bin/env python3
"""
plot_tier2_paper_figures.py
===========================
Tier-2 paper bundle: transfer / plasticity summary / behavior–outcome / world table.

Outputs (under --out-dir):

  Transfer (needs ttd_across_seeds.csv under each rollout root; same as plot_unseen_world_transfer.py)
    transfer_paired_child_<label>.csv, transfer_paired_mother_<label>.csv
    transfer_delta_child_<label>.png, transfer_delta_mother_<label>.png
    transfer_forest_child_<label>.png, transfer_forest_mother_<label>.png
    transfer_forest_*_<label>_summary.csv
    base_genome_means_child.csv, base_genome_means_mother.csv

  Plasticity summary (mean child TTD across all inits × genomes, per stage)
    plasticity_mean_ttd_by_stage.csv
    plasticity_mean_ttd_trajectory.png

  Behavior vs outcome (emergent genomes)
    scatter_pc1_vs_child_ttd_emergent.png
    scatter_maternal_vs_child_ttd_emergent.png
    behavior_outcome_correlations.csv

  World / evaluation parameters (for Methods)
    world_parameters_tier2.txt

Defaults match this repo: base = FinalGenomeRollouts_normal_seen;
unseen = unseen_food30 + unseen_hard (override with --base / --unseen).

Stage CSVs default to extended conditional metrics (same roots as Tier 1).
PCA features default to paper_figures_tier1_extended/signatures_pca_features.csv
(run plot_tier1_paper_figures.py first).
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
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

from plot_unseen_world_transfer import _genome_means, _join_delta, _plot_box_panels, _plot_forest

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
        "axes.grid": True,
        "grid.color": "0.90",
        "grid.linewidth": 0.8,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

PLAST_ORDER = ["E2", "P1", "P2"]
INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_COLORS = {"E2": "#4C78A8", "P1": "#E45756", "P2": "#54A24B"}
INIT_COLORS = {
    "anti_maternal": "#c0392b",
    "random_uniform": "#2980b9",
    "pro_maternal": "#27ae60",
}
PLAST_MARKERS = {"E2": "o", "P1": "s", "P2": "^"}


def _repo_root() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _run_transfer(
    *,
    base: str,
    unseen_specs: list[tuple[str, str]],
    out_dir: str,
    units: str,
    day_step: float,
    bootstrap: int,
    title_prefix: str,
) -> None:
    base = os.path.abspath(base)
    os.makedirs(out_dir, exist_ok=True)
    if units == "days":
        child_col, mother_col = "child_days", "mother_days"
        ylab_c, ylab_m = "Δ child TTD (days)", "Δ mother TTD (days)"
    else:
        child_col, mother_col = "child_ttd_norm", "mother_ttd_norm"
        ylab_c, ylab_m = "Δ child TTD (normalized)", "Δ mother TTD (normalized)"

    base_child = _genome_means(base, value_col=child_col, day_step=day_step)
    base_mother = _genome_means(base, value_col=mother_col, day_step=day_step)
    base_child.to_csv(os.path.join(out_dir, "base_genome_means_child.csv"), index=False)
    base_mother.to_csv(os.path.join(out_dir, "base_genome_means_mother.csv"), index=False)

    prefix = (title_prefix + " — ") if title_prefix else ""

    for label, path in unseen_specs:
        path = os.path.abspath(path)
        if not os.path.isdir(path):
            print(f"[warn] unseen root missing, skip transfer [{label}]: {path}", file=sys.stderr)
            continue
        oc = _genome_means(path, value_col=child_col, day_step=day_step)
        om = _genome_means(path, value_col=mother_col, day_step=day_step)
        merged_c = _join_delta(base_child, oc)
        merged_m = _join_delta(base_mother, om)
        merged_c.to_csv(os.path.join(out_dir, f"transfer_paired_child_{label}.csv"), index=False)
        merged_m.to_csv(os.path.join(out_dir, f"transfer_paired_mother_{label}.csv"), index=False)

        _plot_box_panels(
            merged_c,
            ylabel=ylab_c,
            title=f"{prefix}Δ child TTD: {label} − base (paired by evolve_seed)",
            out_png=os.path.join(out_dir, f"transfer_delta_child_{label}.png"),
        )
        _plot_box_panels(
            merged_m,
            ylabel=ylab_m,
            title=f"{prefix}Δ mother TTD: {label} − base (paired by evolve_seed)",
            out_png=os.path.join(out_dir, f"transfer_delta_mother_{label}.png"),
        )
        summ_c = _plot_forest(
            merged_c,
            ylabel=f"Bootstrap median {ylab_c} (95% CI)",
            title=f"{prefix}Forest: median Δ child TTD ({label} − base)",
            out_png=os.path.join(out_dir, f"transfer_forest_child_{label}.png"),
            n_boot=int(bootstrap),
        )
        summ_m = _plot_forest(
            merged_m,
            ylabel=f"Bootstrap median {ylab_m} (95% CI)",
            title=f"{prefix}Forest: median Δ mother TTD ({label} − base)",
            out_png=os.path.join(out_dir, f"transfer_forest_mother_{label}.png"),
            n_boot=int(bootstrap),
        )
        summ_c.to_csv(os.path.join(out_dir, f"transfer_forest_child_{label}_summary.csv"), index=False)
        summ_m.to_csv(os.path.join(out_dir, f"transfer_forest_mother_{label}_summary.csv"), index=False)


def _per_genome_stage_ttd(df: pd.DataFrame, stage: str) -> pd.DataFrame:
    df = df.copy()
    df["stage"] = stage
    return df.groupby(["stage", "plasticity", "init", "evolve_seed"], as_index=False)[
        "child_ttd_norm"
    ].mean()


def _plasticity_summary(stage_specs: list[tuple[str, str]], out_dir: str) -> None:
    parts = []
    for stage_label, csv_path in stage_specs:
        csv_path = os.path.abspath(csv_path)
        if not os.path.isfile(csv_path):
            print(f"[warn] skip plasticity summary, missing {csv_path}", file=sys.stderr)
            continue
        df = pd.read_csv(csv_path)
        if "child_ttd_norm" not in df.columns:
            continue
        parts.append(_per_genome_stage_ttd(df, stage_label))
    if not parts:
        print("[warn] no stage CSVs for plasticity summary", file=sys.stderr)
        return
    all_pg = pd.concat(parts, ignore_index=True)
    mean_df = (
        all_pg.groupby(["stage", "plasticity"], as_index=False)["child_ttd_norm"]
        .mean()
        .rename(columns={"child_ttd_norm": "mean_ttd"})
    )
    med_df = (
        all_pg.groupby(["stage", "plasticity"], as_index=False)["child_ttd_norm"]
        .median()
        .rename(columns={"child_ttd_norm": "median_ttd"})
    )
    summ = mean_df.merge(med_df, on=["stage", "plasticity"])
    summ.to_csv(os.path.join(out_dir, "plasticity_mean_ttd_by_stage.csv"), index=False)

    stage_order = [s for s, _ in stage_specs if s in set(summ["stage"])]
    if not stage_order:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    xs = np.arange(len(stage_order), dtype=float)
    for plast in PLAST_ORDER:
        ys = []
        for st in stage_order:
            row = summ[(summ["stage"] == st) & (summ["plasticity"] == plast)]
            ys.append(float(row["mean_ttd"].iloc[0]) if len(row) else float("nan"))
        ax.plot(xs, ys, "-o", color=PLAST_COLORS[plast], linewidth=2.0, markersize=7, label=plast)
    ax.set_xticks(xs)
    ax.set_xticklabels(stage_order)
    ax.set_ylabel("mean child TTD (norm)\nacross inits × genomes")
    ax.set_title(
        "Plasticity summary: average child survival metric by stage\n"
        "(each point = mean of per-genome means, pooled over anti / random / pro)",
        fontsize=11,
    )
    ax.legend(title="plasticity", frameon=False)
    fig.tight_layout()
    out_png = os.path.join(out_dir, "plasticity_mean_ttd_trajectory.png")
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _spearman(x: pd.Series, y: pd.Series) -> float:
    """Spearman rho via Pearson on ranks (no scipy)."""
    rx = x.rank(method="average")
    ry = y.rank(method="average")
    return float(rx.corr(ry))


def _behavior_outcome(
    *,
    pca_features_csv: str,
    emergent_conditional_csv: str,
    out_dir: str,
) -> None:
    pca_path = os.path.abspath(pca_features_csv)
    em_path = os.path.abspath(emergent_conditional_csv)
    rows = []
    if os.path.isfile(pca_path):
        pca = pd.read_csv(pca_path)
        need = {"pc1", "child_ttd_norm", "plasticity", "init", "evolve_seed"}
        if need.issubset(pca.columns):
            fig, ax = plt.subplots(figsize=(6.8, 5.4))
            for init in INIT_ORDER:
                for plast in PLAST_ORDER:
                    sub = pca[(pca["init"] == init) & (pca["plasticity"] == plast)]
                    if sub.empty:
                        continue
                    ax.scatter(
                        sub["pc1"],
                        sub["child_ttd_norm"],
                        s=38,
                        c=INIT_COLORS[init],
                        marker=PLAST_MARKERS[plast],
                        edgecolors="white",
                        linewidths=0.5,
                        alpha=0.88,
                        label=f"{init.replace('_', ' ')} / {plast}",
                    )
            rho = _spearman(pca["pc1"], pca["child_ttd_norm"])
            rows.append({"pair": "PC1 vs child_ttd_norm (emergent)", "spearman_rho": rho, "n": len(pca)})
            ax.set_xlabel("PC1 (behavioral signature, Tier 1 extended PCA)")
            ax.set_ylabel("child TTD (normalized, emergent)")
            ax.set_title(
                f"Outcome vs behavioral axis (emergent)\nSpearman ρ = {rho:.3f} (n={len(pca)})",
                fontsize=11,
            )
            ax.legend(bbox_to_anchor=(1.02, 0.5), loc="center left", fontsize=7, frameon=False)
            fig.tight_layout()
            p1 = os.path.join(out_dir, "scatter_pc1_vs_child_ttd_emergent.png")
            fig.savefig(p1, dpi=200, bbox_inches="tight")
            plt.close(fig)
            print("[ok] wrote", p1)
        else:
            print(f"[warn] PCA CSV missing columns {need - set(pca.columns)}: {pca_path}", file=sys.stderr)
    else:
        print(f"[warn] PCA features CSV not found (run Tier 1 first): {pca_path}", file=sys.stderr)

    if os.path.isfile(em_path):
        em = pd.read_csv(em_path)
        if "P_maternal_given_hungry" in em.columns and {"child_ttd_norm", "plasticity", "init", "evolve_seed"}.issubset(
            em.columns
        ):
            agg = em.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[
                ["P_maternal_given_hungry", "child_ttd_norm"]
            ].mean()
            fig, ax = plt.subplots(figsize=(6.8, 5.4))
            for init in INIT_ORDER:
                for plast in PLAST_ORDER:
                    sub = agg[(agg["init"] == init) & (agg["plasticity"] == plast)]
                    if sub.empty:
                        continue
                    ax.scatter(
                        sub["P_maternal_given_hungry"],
                        sub["child_ttd_norm"],
                        s=38,
                        c=INIT_COLORS[init],
                        marker=PLAST_MARKERS[plast],
                        edgecolors="white",
                        linewidths=0.5,
                        alpha=0.88,
                        label=f"{init.replace('_', ' ')} / {plast}",
                    )
            rho2 = _spearman(agg["P_maternal_given_hungry"], agg["child_ttd_norm"])
            rows.append(
                {
                    "pair": "P(maternal|hungry) vs child_ttd_norm (emergent)",
                    "spearman_rho": rho2,
                    "n": len(agg),
                }
            )
            ax.set_xlabel("P(Maternal | child hungry) (emergent rollout mean per genome)")
            ax.set_ylabel("child TTD (normalized)")
            ax.set_title(
                f"Responsiveness vs survival (emergent)\nSpearman ρ = {rho2:.3f} (n={len(agg)})",
                fontsize=11,
            )
            ax.legend(bbox_to_anchor=(1.02, 0.5), loc="center left", fontsize=7, frameon=False)
            fig.tight_layout()
            p2 = os.path.join(out_dir, "scatter_maternal_vs_child_ttd_emergent.png")
            fig.savefig(p2, dpi=200, bbox_inches="tight")
            plt.close(fig)
            print("[ok] wrote", p2)
        else:
            print(f"[warn] emergent conditional CSV missing columns: {em_path}", file=sys.stderr)
    else:
        print(f"[warn] emergent conditional CSV not found: {em_path}", file=sys.stderr)

    if rows:
        pd.DataFrame(rows).to_csv(os.path.join(out_dir, "behavior_outcome_correlations.csv"), index=False)
        print("[ok] wrote", os.path.join(out_dir, "behavior_outcome_correlations.csv"))


def _world_parameters(out_dir: str) -> None:
    text = """World and evaluation parameters (Tier 2 reference for Methods)
================================================================

Rollout / headless evaluation (matches run_headless_rollout_emergence_final_genomes.sh defaults):
  Grid: 15 x 15
  Food spawn: every 15 ticks, 1 food per event
  World presets: normal = 1 threat; hard = 2 threats (unseen_hard)
  Episode length: max_ticks = 1000 (child_ttd_norm = child_death_tick / max_tick in logs)
  day_step: 100 ticks per simulated day (used when converting to days in transfer script)

Passive lower bound (baseline_passive_lower_bound.py):
  Mother forced passive (no effective actions); same grid/food/threats as rollouts when flags match.

Active upper reference (single genome):
  One fixed pro_maternal final_genome.json evaluated under the same world flags as rollouts.

Transfer roots (defaults in plot_tier2_paper_figures.py):
  base:  FinalGenomeRollouts_normal_seen
  unseen_food30: FinalGenomeRollouts_unseen_food30  (scarcer food than base if configured that way in rollout)
  unseen_hard:   FinalGenomeRollouts_unseen_hard    (more threats than normal)

Adjust any of these in the paper if your actual run scripts used different flags.
"""
    path = os.path.join(out_dir, "world_parameters_tier2.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print("[ok] wrote", path)


def _parse_unseen(spec: str) -> tuple[str, str]:
    if "=" not in spec:
        raise SystemExit(f"--unseen must be label=path, got: {spec}")
    a, b = spec.split("=", 1)
    return a.strip(), b.strip()


def main(argv: Iterable[str] | None = None) -> int:
    root = _repo_root()
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--out-dir",
        default=os.path.join(root, "paper_figures_tier2"),
        help="Output directory (default: <repo>/paper_figures_tier2)",
    )
    ap.add_argument(
        "--base",
        default=os.path.join(root, "FinalGenomeRollouts_normal_seen"),
        help="Training-world rollout root (ttd_across_seeds.csv per genome)",
    )
    ap.add_argument(
        "--unseen",
        action="append",
        default=[],
        help="Repeatable label=path (default: unseen_food30 and unseen_hard under repo if --unseen omitted)",
    )
    ap.add_argument("--no-transfer", action="store_true", help="Skip unseen transfer plots.")
    ap.add_argument("--units", choices=["norm", "days"], default="norm")
    ap.add_argument("--day-step", type=float, default=100.0)
    ap.add_argument("--bootstrap", type=int, default=4000)
    ap.add_argument("--title-prefix", default="")
    ap.add_argument(
        "--stage-csv",
        action="append",
        default=[],
        help="Repeatable label=path for plasticity summary (default: StageRollouts_normal extended metrics)",
    )
    ap.add_argument(
        "--pca-features-csv",
        default=os.path.join(root, "paper_figures_tier1_extended", "signatures_pca_features.csv"),
        help="From plot_tier1_paper_figures.py (extended PCA)",
    )
    ap.add_argument(
        "--emergent-conditional-csv",
        default=os.path.join(
            root,
            "StageRollouts_normal/figures_conditional_extended/conditional_metrics_emergent.csv",
        ),
        help="Emergent conditional_metrics CSV (extended recommended)",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    unseen_specs: list[tuple[str, str]] = []
    if args.unseen:
        for s in args.unseen:
            unseen_specs.append(_parse_unseen(s))
    else:
        unseen_specs = [
            ("unseen_food30", os.path.join(root, "FinalGenomeRollouts_unseen_food30")),
            ("unseen_hard", os.path.join(root, "FinalGenomeRollouts_unseen_hard")),
        ]

    if not args.no_transfer:
        _run_transfer(
            base=args.base,
            unseen_specs=unseen_specs,
            out_dir=out_dir,
            units=args.units,
            day_step=args.day_step,
            bootstrap=args.bootstrap,
            title_prefix=args.title_prefix,
        )

    if args.stage_csv:
        stage_specs = [_parse_unseen(s) for s in args.stage_csv]
    else:
        stage_specs = [
            (
                "initial",
                os.path.join(
                    root, "StageRollouts_normal/figures_conditional_extended/conditional_metrics_initial.csv"
                ),
            ),
            (
                "transient",
                os.path.join(
                    root, "StageRollouts_normal/figures_conditional_extended/conditional_metrics_transient.csv"
                ),
            ),
            (
                "emergent",
                os.path.join(
                    root, "StageRollouts_normal/figures_conditional_extended/conditional_metrics_emergent.csv"
                ),
            ),
        ]
    _plasticity_summary(stage_specs, out_dir)

    _behavior_outcome(
        pca_features_csv=args.pca_features_csv,
        emergent_conditional_csv=args.emergent_conditional_csv,
        out_dir=out_dir,
    )

    _world_parameters(out_dir)

    idx = os.path.join(out_dir, "tier2_index.txt")
    with open(idx, "w", encoding="utf-8") as f:
        f.write("Tier 2 outputs in this directory:\n")
        for fn in sorted(os.listdir(out_dir)):
            f.write(f"  {fn}\n")
    print("[ok] wrote", idx)
    return 0


if __name__ == "__main__":
    sys.exit(main())
