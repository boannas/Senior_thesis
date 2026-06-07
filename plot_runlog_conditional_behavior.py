#!/usr/bin/env python3
"""
plot_runlog_conditional_behavior.py
===================================
Compute *conditional* maternal-behavior metrics from RunLogger-style headless
rollout logs (run_*.csv) and plot them by init × plasticity, optionally across
multiple "stages" (initial / transient / emergent).

Why this exists
---------------
Unconditional motivation fractions (e.g. mean `m0_sel_Care`) hide whether the
mother is actually *responsive* to the child's state. The metrics below condition
on the child's state, so an "emergent" mother will score high while a noisy
anti-maternal start scores low even if her unconditional Care fraction is fine.

Metrics (computed per run_*.csv, over ticks where m0_energy & c0_hunger are finite)
----------------------------------------------------------------------------------
  P(Maternal | child hungry)   = P(sel in {Care, Protect} | c0_hunger > tau_hunger)
  P(Forage   | child hungry)   = P(sel == Forage          | c0_hunger > tau_hunger)
  P(Protect  | mother afraid)  = P(sel == Protect         | m0_fear_threat > tau_fear)
  closeness_mean               = mean(m0_closeness_child) on alive ticks
  injury_mean                  = mean(c0_injury)        on alive ticks
  fear_mean                    = mean(m0_fear_threat)   on alive ticks

Extended metrics (--extended)
------------------------------
  Per-motivation P(sel=M | condition) for M in {Forage,Care,Self,Protect} with conditions:
    child hungry (c0_hunger > tau_hunger)
    child cold   (c0_warmth < tau; 0–100 scale, 50 ideal — default marks clearly sub-ideal cold)
    child hurt   (c0_injury > tau_injury)
    mother fear  (m0_fear_threat > tau_fear)
    mother low energy (m0_energy < tau_energy_low; 0–100)
    mother high stress (m0_stress > tau_stress_high; 0–100)
  Plus mean mother physiology on alive ticks: energy, fatigue, bonding, stress (if columns exist).

Interpreting P(X | Y) in the logs
-------------------------------
Each row is one tick. **P( motivation X | condition Y )** is the fraction of ticks where **Y is true** (and both agents are “alive” per the script) on which the **selected** motivation that tick was X. Here X is one of {Forage, Care, Self, Protect} from the one-hot `m0_sel_*` columns.

So **P(Forage | child cold)** answers: when the child was already in a “cold” band, how often did the mother’s *chosen* action category Forage win the argmax that tick? It is **not** a causal effect; it is descriptive co-occurrence under your threshold for “cold.”

State scales in CSV (same as `run_logger` / agents)
--------------------------------------------------
Child `c0_hunger`, `c0_warmth`, `c0_injury` and mother `m0_energy`, `m0_stress`, `m0_fear_threat` are on **roughly 0–100** scales in the rollout logs (warmth is **50 = comfortable**; hunger rises toward starvation; injury toward lethal). Extended thresholds must use those units: an old bug used sub-1 defaults for warmth/energy, which made “cold” / “low energy” masks almost always empty.

Usage
-----
Single root (boxplot grid by init x plasticity, one figure per metric):

  python3 plot_runlog_conditional_behavior.py \\
      --root FinalGenomeRollouts_normal_seen

  # All extended combinations (many PNGs):
  python3 plot_runlog_conditional_behavior.py --root FinalGenomeRollouts_normal_seen --extended

Multi-stage (one panel per stage in each figure, plus a per-init line summary):

  python3 plot_runlog_conditional_behavior.py \\
      --stage-root initial=StageRollouts_normal/initial \\
      --stage-root transient=StageRollouts_normal/transient \\
      --stage-root emergent=StageRollouts_normal/emergent \\
      --out-dir StageRollouts_normal/figures_conditional \\
      --extended

  # Only Forage|hungry, Care|cold, Protect|injured + mean hunger/warmth/injury — trajectories:
  python3 plot_runlog_conditional_behavior.py \\
      --stage-root initial=... --stage-root transient=... --stage-root emergent=... \\
      --preset child_state_paired --trajectory-only --out-dir figures_paired_trajectories
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
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


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


INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]
COLORS = {"E2": "#4C78A8", "P1": "#E45756", "P2": "#54A24B"}

# Defaults for --extended thresholds (run_logger / agents use ~0–100 for these states).
# Warmth: 50 is ideal (see ChildAgent); "cold" = below-comfort band toward freezing.
DEFAULT_WARMTH_COLD_TAU = 47.0
# Injury: 0 healthy → 100 lethal; pick a low tau to tag noticeable harm in typical rollouts.
DEFAULT_INJURY_TAU = 1.0
# Mother energy: same order as hunger; "low" = below half tank (tunable).
DEFAULT_ENERGY_LOW_TAU = 50.0
# Mother stress: elevated band on 0–100 (tunable if masks are too rare/frequent).
DEFAULT_STRESS_HIGH_TAU = 40.0

RUN_RE = re.compile(
    r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$"
)


CORE_METRICS: list[tuple[str, str]] = [
    ("P_maternal_given_hungry", "P(Maternal | child hungry)"),
    ("P_forage_given_hungry", "P(Forage | child hungry)"),
    ("P_protect_given_fear", "P(Protect | mother afraid)"),
    ("closeness_mean_alive", "mean closeness_child (alive ticks)"),
    ("injury_mean_alive", "mean c0_injury (alive ticks)"),
    ("fear_mean_alive", "mean m0_fear_threat (alive ticks)"),
    ("child_ttd_norm", "child TTD (normalized)"),
    ("mother_ttd_norm", "mother TTD (normalized)"),
]

MOTIVATIONS = ("Forage", "Care", "Self", "Protect")

# Subset for Exp 3: motivation|child-state pairings + mean child physiology (trajectory figures).
CHILD_STATE_PAIRED_METRICS: list[tuple[str, str]] = [
    ("P_sel_Forage_given_child_hungry", "P(Forage | child hungry)"),
    ("P_sel_Care_given_child_cold", "P(Care | child cold, c0_warmth < τ)"),
    ("P_sel_Protect_given_child_injured", "P(Protect | child injured)"),
    ("hunger_mean_alive", "mean c0_hunger (alive ticks)"),
    ("warmth_mean_alive", "mean c0_warmth (alive ticks)"),
    ("injury_mean_alive", "mean c0_injury (alive ticks)"),
]
CHILD_STATE_PAIRED_KEYS = {k for k, _ in CHILD_STATE_PAIRED_METRICS}


def _alive_mask(df: "pd.DataFrame") -> np.ndarray:
    m = pd.to_numeric(df.get("m0_energy"), errors="coerce").to_numpy(dtype=float)
    c = pd.to_numeric(df.get("c0_hunger"), errors="coerce").to_numpy(dtype=float)
    return np.isfinite(m) & np.isfinite(c)


def _p_sel_given_mask(sel: np.ndarray, mask: np.ndarray) -> tuple[float, int]:
    n = int(mask.sum())
    if n <= 0:
        return float("nan"), 0
    return float(np.mean(sel[mask])), n


def _load_one_run(
    csv_path: str,
    *,
    hunger_tau: float,
    fear_tau: float,
    extended: bool,
    warmth_cold_tau: float,
    injury_tau: float,
    energy_low_tau: float,
    stress_high_tau: float,
) -> dict | None:
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    needed = (
        "m0_energy",
        "c0_hunger",
        "m0_sel_Care",
        "m0_sel_Protect",
        "m0_sel_Forage",
        "m0_sel_Self",
        "m0_fear_threat",
        "m0_closeness_child",
        "c0_injury",
    )
    for col in needed:
        if col not in df.columns:
            return None

    alive = _alive_mask(df)
    n_alive = int(alive.sum())
    if n_alive <= 0:
        return None

    hunger = pd.to_numeric(df["c0_hunger"], errors="coerce").to_numpy(dtype=float)
    fear = pd.to_numeric(df["m0_fear_threat"], errors="coerce").to_numpy(dtype=float)
    closeness = pd.to_numeric(df["m0_closeness_child"], errors="coerce").to_numpy(dtype=float)
    injury = pd.to_numeric(df["c0_injury"], errors="coerce").to_numpy(dtype=float)
    energy = pd.to_numeric(df["m0_energy"], errors="coerce").to_numpy(dtype=float)

    sel_care = pd.to_numeric(df["m0_sel_Care"], errors="coerce").fillna(0).to_numpy(dtype=float)
    sel_protect = pd.to_numeric(df["m0_sel_Protect"], errors="coerce").fillna(0).to_numpy(dtype=float)
    sel_forage = pd.to_numeric(df["m0_sel_Forage"], errors="coerce").fillna(0).to_numpy(dtype=float)
    sel_self = pd.to_numeric(df["m0_sel_Self"], errors="coerce").fillna(0).to_numpy(dtype=float)
    sel_by_mot = {"Forage": sel_forage, "Care": sel_care, "Self": sel_self, "Protect": sel_protect}

    out: dict = {"path": csv_path, "n_alive_ticks": n_alive}

    # P(maternal | hungry)
    mask_h = alive & np.isfinite(hunger) & (hunger > hunger_tau)
    n_h = int(mask_h.sum())
    if n_h > 0:
        out["P_maternal_given_hungry"] = float(np.mean((sel_care + sel_protect)[mask_h]))
        out["P_forage_given_hungry"] = float(np.mean(sel_forage[mask_h]))
    else:
        out["P_maternal_given_hungry"] = float("nan")
        out["P_forage_given_hungry"] = float("nan")
    out["n_hungry_alive_ticks"] = n_h

    # P(protect | fear)
    mask_f = alive & np.isfinite(fear) & (fear > fear_tau)
    n_f = int(mask_f.sum())
    if n_f > 0:
        out["P_protect_given_fear"] = float(np.mean(sel_protect[mask_f]))
    else:
        out["P_protect_given_fear"] = float("nan")
    out["n_fear_alive_ticks"] = n_f

    out["closeness_mean_alive"] = float(np.nanmean(closeness[alive])) if n_alive else float("nan")
    out["hunger_mean_alive"] = float(np.nanmean(hunger[alive])) if n_alive else float("nan")
    out["injury_mean_alive"] = float(np.nanmean(injury[alive])) if n_alive else float("nan")
    out["fear_mean_alive"] = float(np.nanmean(fear[alive])) if n_alive else float("nan")

    # TTD (normalized by max tick observed). First non-finite tick = death time.
    if "tick" in df.columns:
        ticks = pd.to_numeric(df["tick"], errors="coerce").to_numpy(dtype=float)
        max_tick = float(np.nanmax(ticks)) if np.isfinite(np.nanmax(ticks)) else float(len(ticks))
    else:
        max_tick = float(len(df))

    def _first_dead(arr: np.ndarray) -> float:
        bad = ~np.isfinite(arr)
        idx = np.argmax(bad) if bad.any() else len(arr)
        # ticks in run_logger are 1-based; +1 to match plot_runlog_ttd_across_seeds.py
        return float(idx + 1) if bad.any() else float(max_tick)

    c_ttd = _first_dead(pd.to_numeric(df["c0_hunger"], errors="coerce").to_numpy(dtype=float))
    m_ttd = _first_dead(pd.to_numeric(df["m0_energy"], errors="coerce").to_numpy(dtype=float))
    if max_tick > 0:
        out["child_ttd_norm"] = float(c_ttd / max_tick)
        out["mother_ttd_norm"] = float(m_ttd / max_tick)
    else:
        out["child_ttd_norm"] = float("nan")
        out["mother_ttd_norm"] = float("nan")

    if not extended:
        return out

    warmth = None
    if "c0_warmth" in df.columns:
        warmth = pd.to_numeric(df["c0_warmth"], errors="coerce").to_numpy(dtype=float)
    stress = None
    if "m0_stress" in df.columns:
        stress = pd.to_numeric(df["m0_stress"], errors="coerce").to_numpy(dtype=float)

    z = np.zeros(alive.shape[0], dtype=bool)
    conditions: list[tuple[str, np.ndarray]] = [("child_hungry", mask_h)]
    if warmth is not None:
        conditions.append(("child_cold", alive & np.isfinite(warmth) & (warmth < warmth_cold_tau)))
    else:
        conditions.append(("child_cold", z))
    conditions.append(("child_injured", alive & np.isfinite(injury) & (injury > injury_tau)))
    conditions.append(("mother_fear_hi", mask_f))
    conditions.append(("mother_energy_lo", alive & np.isfinite(energy) & (energy < energy_low_tau)))
    if stress is not None:
        conditions.append(("mother_stress_hi", alive & np.isfinite(stress) & (stress > stress_high_tau)))
    else:
        conditions.append(("mother_stress_hi", z))

    for cond_key, m in conditions:
        out[f"n_ticks_{cond_key}"] = int(m.sum())
        for mot in MOTIVATIONS:
            key = f"P_sel_{mot}_given_{cond_key}"
            v, _ = _p_sel_given_mask(sel_by_mot[mot], m)
            out[key] = v

    if warmth is not None:
        out["warmth_mean_alive"] = float(np.nanmean(warmth[alive])) if n_alive else float("nan")
    else:
        out["warmth_mean_alive"] = float("nan")
    if stress is not None:
        out["stress_mean_alive"] = float(np.nanmean(stress[alive])) if n_alive else float("nan")
    else:
        out["stress_mean_alive"] = float("nan")

    out["energy_mean_alive"] = float(np.nanmean(energy[alive])) if n_alive else float("nan")

    for col, outkey in (
        ("m0_fatigue", "fatigue_mean_alive"),
        ("m0_bonding", "bonding_mean_alive"),
    ):
        if col in df.columns:
            arr = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
            out[outkey] = float(np.nanmean(arr[alive])) if n_alive else float("nan")

    return out


def _extended_metric_list() -> list[tuple[str, str]]:
    """Human-readable labels for all extended-only columns (for plotting)."""
    rows: list[tuple[str, str]] = []
    conds = [
        ("child_hungry", "child hungry"),
        ("child_cold", "child cold (c0_warmth < τ, 0–100; 50 ideal)"),
        ("child_injured", "child injured"),
        ("mother_fear_hi", "mother fear high"),
        ("mother_energy_lo", "mother energy low"),
        ("mother_stress_hi", "mother stress high"),
    ]
    for ck, clab in conds:
        for mot in MOTIVATIONS:
            key = f"P_sel_{mot}_given_{ck}"
            rows.append((key, f"P({mot} | {clab})"))
    rows.append(("warmth_mean_alive", "mean c0_warmth (alive ticks)"))
    rows.append(("energy_mean_alive", "mean m0_energy (alive ticks)"))
    rows.append(("stress_mean_alive", "mean m0_stress (alive ticks)"))
    rows.append(("fatigue_mean_alive", "mean m0_fatigue (alive ticks)"))
    rows.append(("bonding_mean_alive", "mean m0_bonding (alive ticks)"))
    return rows


def _metrics_for_plot(extended: bool) -> list[tuple[str, str]]:
    out = list(CORE_METRICS)
    if extended:
        out.extend(_extended_metric_list())
    return out


def _filter_metrics(
    metrics: list[tuple[str, str]],
    *,
    only_keys: set[str] | None,
    preset: str | None,
) -> list[tuple[str, str]]:
    if preset == "child_state_paired":
        return list(CHILD_STATE_PAIRED_METRICS)
    if not only_keys:
        return metrics
    by_key = {k: lab for k, lab in metrics}
    missing = sorted(only_keys - set(by_key.keys()))
    if missing:
        print(
            "[warn] unknown or unavailable --metrics keys (need --extended for P_sel_*):",
            ", ".join(missing),
            file=sys.stderr,
        )
    out: list[tuple[str, str]] = []
    for k in only_keys:
        if k in by_key:
            out.append((k, by_key[k]))
    return out


def _collect_root(
    root: str,
    *,
    hunger_tau: float,
    fear_tau: float,
    extended: bool,
    warmth_cold_tau: float,
    injury_tau: float,
    energy_low_tau: float,
    stress_high_tau: float,
) -> "pd.DataFrame":
    paths = sorted(glob.glob(os.path.join(root, "*", "run_*.csv")))
    if not paths:
        raise SystemExit(f"No run_*.csv under {root}/*/")
    rows = []
    skipped = 0
    for p in paths:
        rd = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(rd)
        if not m:
            skipped += 1
            continue
        rec = _load_one_run(
            p,
            hunger_tau=hunger_tau,
            fear_tau=fear_tau,
            extended=extended,
            warmth_cold_tau=warmth_cold_tau,
            injury_tau=injury_tau,
            energy_low_tau=energy_low_tau,
            stress_high_tau=stress_high_tau,
        )
        if rec is None:
            skipped += 1
            continue
        rec["run_dir"] = rd
        rec["plasticity"] = m.group("plast")
        rec["init"] = m.group("init")
        rec["evolve_seed"] = int(m.group("seed"))
        rows.append(rec)
    if not rows:
        raise SystemExit(f"No usable run logs under {root}. (skipped={skipped})")
    df = pd.DataFrame(rows)
    print(f"[ok] {root}: loaded {len(df)} runs (skipped={skipped})")
    return df


def _styled_boxplot(ax, data, labels, color):
    bp = ax.boxplot(
        data,
        labels=labels,
        patch_artist=True,
        widths=0.6,
        showmeans=True,
        medianprops=dict(color="#F28E2B", linewidth=1.6),
        meanprops=dict(marker="^", markerfacecolor="#2ca02c", markeredgecolor="#2ca02c", markersize=6),
        whiskerprops=dict(color="0.25", linewidth=0.8),
        capprops=dict(color="0.25", linewidth=0.8),
        flierprops=dict(marker="o", markerfacecolor="none", markeredgecolor="0.2", markersize=4, linestyle="none", alpha=0.18),
    )
    for patch in bp["boxes"]:
        patch.set_facecolor(color)
        patch.set_alpha(0.16)
        patch.set_edgecolor(color)
        patch.set_linewidth(1.0)


def _plot_metric_box_panels(df: "pd.DataFrame", *, value_col: str, ylabel: str, title: str, out_png: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharey=True)
    rng = np.random.default_rng(0)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        data = []
        for init in INIT_ORDER:
            vals = pd.to_numeric(sub.loc[sub["init"] == init, value_col], errors="coerce").dropna().to_numpy()
            data.append(vals)
        if not any(len(d) for d in data):
            ax.set_visible(False)
            continue
        labels = [s.replace("_", " ") for s in INIT_ORDER]
        _styled_boxplot(ax, data, labels, color=COLORS[plast])
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.05, size=len(vals))
            ax.scatter(x, vals, s=10, alpha=0.18, color=COLORS[plast], edgecolors="none", zorder=3)
        ax.set_title(plast, fontsize=12)
    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)


def _plot_stage_trajectories(stage_dfs: dict[str, "pd.DataFrame"], *, value_col: str, ylabel: str, title: str, out_png: str, stage_order: list[str]) -> None:
    """
    One figure with 3 panels (E2/P1/P2). x = stage, y = metric.
    Three lines (init types), mean ± IQR across genomes (one mean per genome).
    """
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharey=True)
    colors_init = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}
    for ax, plast in zip(axes, PLAST_ORDER):
        for init in INIT_ORDER:
            xs, mids, los, his = [], [], [], []
            for i, stage in enumerate(stage_order):
                df = stage_dfs.get(stage)
                if df is None or df.empty:
                    continue
                sub = df[(df["plasticity"] == plast) & (df["init"] == init)]
                # collapse to one number per genome
                per_genome = sub.groupby("evolve_seed", as_index=False)[value_col].mean()
                vals = per_genome[value_col].dropna().to_numpy(dtype=float)
                if len(vals) == 0:
                    continue
                xs.append(i)
                mids.append(float(np.median(vals)))
                los.append(float(np.percentile(vals, 25)))
                his.append(float(np.percentile(vals, 75)))
            if not xs:
                continue
            xs = np.asarray(xs, dtype=float)
            mids = np.asarray(mids, dtype=float)
            los = np.asarray(los, dtype=float)
            his = np.asarray(his, dtype=float)
            ax.fill_between(xs, los, his, color=colors_init[init], alpha=0.18, linewidth=0)
            ax.plot(xs, mids, "-o", color=colors_init[init], linewidth=1.8, markersize=5, label=init.replace("_", " "))
        ax.set_title(plast, fontsize=12)
        ax.set_xticks(range(len(stage_order)))
        ax.set_xticklabels(stage_order)
    axes[0].set_ylabel(ylabel)
    axes[-1].legend(loc="best", fontsize=9, frameon=False)
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=None, help="Single root with <plast>_<init>_seedN/run_*.csv")
    ap.add_argument(
        "--stage-root",
        action="append",
        default=[],
        help="Repeatable: '<label>=<root>'. Multiple stages plotted as trajectories.",
    )
    ap.add_argument("--out-dir", default=None, help="Output directory (default: <root>/figures_conditional or first stage root's parent)")
    ap.add_argument("--hunger-tau", type=float, default=50.0, help="Threshold on c0_hunger for 'child hungry' (default 50)")
    ap.add_argument("--fear-tau", type=float, default=10.0, help="Threshold on m0_fear_threat for 'mother afraid' (default 10)")
    ap.add_argument("--title-prefix", default="")
    ap.add_argument(
        "--extended",
        action="store_true",
        help="Add per-motivation conditionals, extra child/mother masks, and mother physiology means (many extra figures).",
    )
    ap.add_argument(
        "--extended-subdir",
        default="extended",
        help="Subfolder under --out-dir for extended-only PNGs (default: extended). Core figures stay in --out-dir.",
    )
    ap.add_argument(
        "--warmth-cold-tau",
        type=float,
        default=DEFAULT_WARMTH_COLD_TAU,
        help="Child cold mask: c0_warmth < this (0–100; 50 ideal). Default ~47 tags sub-ideal cold.",
    )
    ap.add_argument(
        "--injury-tau",
        type=float,
        default=DEFAULT_INJURY_TAU,
        help="Child injured mask: c0_injury > this (0–100 lethal scale). Raise if too many ticks qualify.",
    )
    ap.add_argument(
        "--energy-low-tau",
        type=float,
        default=DEFAULT_ENERGY_LOW_TAU,
        help="Mother low energy: m0_energy < this (0–100).",
    )
    ap.add_argument(
        "--stress-high-tau",
        type=float,
        default=DEFAULT_STRESS_HIGH_TAU,
        help="Mother high stress: m0_stress > this (0–100). Lower if this mask is too rare.",
    )
    ap.add_argument(
        "--metrics",
        default=None,
        help="Comma-separated metric column names to plot only (e.g. P_sel_Forage_given_child_hungry,warmth_mean_alive). "
        "Implies --extended if any P_sel_* key is listed.",
    )
    ap.add_argument(
        "--preset",
        choices=("child_state_paired",),
        default=None,
        help="Plot a fixed small set: Forage|hungry, Care|cold, Protect|injured + mean hunger/warmth/injury.",
    )
    ap.add_argument(
        "--trajectory-only",
        action="store_true",
        help="With --stage-root: write only *_trajectory.png (skip per-stage boxplots).",
    )
    args = ap.parse_args()

    if not args.root and not args.stage_root:
        print("[error] provide --root and/or one or more --stage-root label=path", file=sys.stderr)
        return 2

    prefix = (args.title_prefix + " — ") if args.title_prefix else ""
    only_keys: set[str] | None = None
    if args.metrics:
        only_keys = {k.strip() for k in str(args.metrics).split(",") if k.strip()}
    need_extended = bool(args.extended) or bool(args.preset == "child_state_paired") or bool(
        only_keys and any(k.startswith("P_sel_") for k in only_keys)
    )
    metrics_to_plot = _filter_metrics(
        _metrics_for_plot(need_extended),
        only_keys=only_keys,
        preset=args.preset,
    )
    if not metrics_to_plot:
        print("[error] no metrics selected (--metrics / --preset)", file=sys.stderr)
        return 2
    core_keys = {k for k, _ in CORE_METRICS}

    load_kw = dict(
        hunger_tau=args.hunger_tau,
        fear_tau=args.fear_tau,
        extended=need_extended,
        warmth_cold_tau=args.warmth_cold_tau,
        injury_tau=args.injury_tau,
        energy_low_tau=args.energy_low_tau,
        stress_high_tau=args.stress_high_tau,
    )

    if args.root and not args.trajectory_only:
        root = os.path.abspath(args.root)
        out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "figures_conditional")
        os.makedirs(out_dir, exist_ok=True)
        ext_dir = os.path.join(out_dir, args.extended_subdir) if need_extended else None
        if ext_dir:
            os.makedirs(ext_dir, exist_ok=True)
        df = _collect_root(root, **load_kw)
        df.to_csv(os.path.join(out_dir, "conditional_metrics_single.csv"), index=False)
        for key, label in metrics_to_plot:
            dest = ext_dir if need_extended and key not in core_keys else out_dir
            _plot_metric_box_panels(
                df,
                value_col=key,
                ylabel=label,
                title=f"{prefix}{label} (single root)",
                out_png=os.path.join(dest, f"cond_{key}_single.png"),
            )

    if args.stage_root:
        stages: dict[str, str] = {}
        for spec in args.stage_root:
            if "=" not in spec:
                print(f"[error] bad --stage-root spec (need label=path): {spec}", file=sys.stderr)
                return 2
            label, path = spec.split("=", 1)
            stages[label.strip()] = os.path.abspath(path.strip())

        stage_order = list(stages.keys())
        stage_dfs: dict[str, pd.DataFrame] = {}
        any_path = next(iter(stages.values()))
        out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(os.path.dirname(any_path), "figures_conditional_stages")
        os.makedirs(out_dir, exist_ok=True)
        ext_dir = os.path.join(out_dir, args.extended_subdir) if need_extended else None
        if ext_dir:
            os.makedirs(ext_dir, exist_ok=True)

        for label, path in stages.items():
            df = _collect_root(path, **load_kw)
            df["stage"] = label
            stage_dfs[label] = df
            df.to_csv(os.path.join(out_dir, f"conditional_metrics_{label}.csv"), index=False)

        # Per-stage boxplots (one figure per metric per stage)
        if not args.trajectory_only:
            for label, df in stage_dfs.items():
                for key, ml in metrics_to_plot:
                    dest = ext_dir if need_extended and key not in core_keys else out_dir
                    _plot_metric_box_panels(
                        df,
                        value_col=key,
                        ylabel=ml,
                        title=f"{prefix}{ml} — {label}",
                        out_png=os.path.join(dest, f"cond_{key}_box_{label}.png"),
                    )

        # Stage trajectories (one figure per metric)
        for key, ml in metrics_to_plot:
            if args.preset == "child_state_paired":
                dest = out_dir
            else:
                dest = ext_dir if need_extended and key not in core_keys else out_dir
            _plot_stage_trajectories(
                stage_dfs,
                value_col=key,
                ylabel=ml,
                title=f"{prefix}{ml} across stages (median ± IQR per genome)",
                out_png=os.path.join(dest, f"cond_{key}_trajectory.png"),
                stage_order=stage_order,
            )
        print(f"[ok] wrote {len(metrics_to_plot)} trajectory figure(s) under {out_dir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
