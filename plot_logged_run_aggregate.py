"""
Run N headless rollouts (different RNG seeds), then plot mean ± √(variance)
across seeds for the same panels as plot_logged_run (states, motivations,
selected motivation, overall deficit).

Also writes ``survival_agg.png``: fraction of runs in which mother/child are
still alive at each time step, plus mean/median death-tick summary.

All figures share a multi-line suptitle (optional ``--label``, seed count,
variance ddof, survival summary). Subplots use x-label "Time step (tick)".
Dashed vertical lines mark mean mother (blue) and child (red) death ticks.

Usage:
  python plot_logged_run_aggregate.py --genome final_genome.json --n-seeds 32 \\
      --seed-base 5000 --max-ticks 800 --out figures_agg/ --label "final_genome"

Requires: pandas, numpy, matplotlib (same as plot_logged_run).
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

try:
    import matplotlib.lines as mlines
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    print("Requires pandas and matplotlib:", e, file=sys.stderr)
    sys.exit(1)

from headless_rollout_log import (
    DEFAULT_DAY_STEP,
    DEFAULT_FOOD_AT_START,
    DEFAULT_FOOD_SPAWN_INTERVAL,
    DEFAULT_FOOD_SPAWN_N,
    DEFAULT_GRID_H,
    DEFAULT_GRID_W,
    DEFAULT_MAX_TICKS,
    DEFAULT_NUM_CHILDREN,
    DEFAULT_NUM_MOTHERS,
    DEFAULT_NUM_THREATS,
    _world_from_genome_and_seed,
    collect_rollout_dataframe,
)
from plot_logged_run import ALL_CHILD_STATES, ALL_MOTHER_STATES, _agent_indices


def _stack_column(dfs: list[pd.DataFrame], col: str, T: int) -> np.ndarray:
    """Shape (n_seeds, T); pad with nan if run ended early or missing column."""
    mat = np.full((len(dfs), T), np.nan, dtype=float)
    for i, df in enumerate(dfs):
        if col not in df.columns:
            continue
        s = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        L = min(len(s), T)
        if L:
            mat[i, :L] = s[:L]
    return mat


def _nanmean_nanvar(mat: np.ndarray, *, ddof: int) -> tuple[np.ndarray, np.ndarray]:
    mean = np.nanmean(mat, axis=0)
    var = np.nanvar(mat, axis=0, ddof=ddof)
    return mean, var


def _band(ax, x, mean, var, color: str, label: str, ddof: int) -> None:
    std = np.sqrt(np.maximum(var, 0.0))
    lo = mean - std
    hi = mean + std
    ax.fill_between(x, lo, hi, color=color, alpha=0.22, linewidth=0)
    ax.plot(x, mean, color=color, label=label, linewidth=1.2)


def _first_dead_tick(series: np.ndarray) -> int:
    """First index where value is non-finite (agent logged as dead); else len(series)."""
    for i, v in enumerate(series):
        if not np.isfinite(v):
            return int(i)
    return int(len(series))


def _death_ticks_for_column(dfs: list[pd.DataFrame], col: str) -> np.ndarray:
    ticks = []
    for df in dfs:
        if col not in df.columns:
            ticks.append(float(len(df)))
            continue
        s = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        ticks.append(float(_first_dead_tick(s)))
    return np.array(ticks, dtype=float)


def _fraction_alive_series(dfs: list[pd.DataFrame], col: str, T: int) -> tuple[np.ndarray, np.ndarray]:
    """Fraction of seeds with finite ``col`` at each tick (1 = alive in log); variance across seeds."""
    mat = _stack_column(dfs, col, T)
    alive = np.isfinite(mat).astype(float)
    mean = np.nanmean(alive, axis=0)
    var = np.nanvar(alive, axis=0, ddof=0)
    return mean, var


def _survival_summary_lines(
    dfs: list[pd.DataFrame],
    *,
    T: int,
    mi: int = 0,
    ki: int = 0,
) -> tuple[str, str, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (suptitle_line, column_note, x, frac_m_alive, frac_c_alive, death_ticks_m, death_ticks_c)."""
    x = np.arange(T, dtype=float)
    mcol = f"m{mi}_energy"
    ccol = f"c{ki}_hunger"
    dm = _death_ticks_for_column(dfs, mcol)
    dc = _death_ticks_for_column(dfs, ccol)
    frac_m, _ = _fraction_alive_series(dfs, mcol, T)
    frac_c, _ = _fraction_alive_series(dfs, ccol, T)
    line = (
        f"Survival: mean death tick mother {np.mean(dm):.1f}±{np.std(dm):.1f}, "
        f"child {np.mean(dc):.1f}±{np.std(dc):.1f} (median M {np.median(dm):.0f}, C {np.median(dc):.0f})"
    )
    short = f"P(alive): {mcol}, {ccol}"
    return line, short, x, frac_m, frac_c, dm, dc


def _apply_xlabel_time_step(ax) -> None:
    ax.set_xlabel("Time step (tick)")


def _mark_mean_death_ticks(ax, dm: np.ndarray, dc: np.ndarray) -> None:
    """Faint vertical lines at mean death tick (mother / child)."""
    mx = float(np.mean(dm))
    cx = float(np.mean(dc))
    ax.axvline(mx, color="steelblue", linestyle="--", alpha=0.35, linewidth=1.0, zorder=0)
    ax.axvline(cx, color="indianred", linestyle="--", alpha=0.35, linewidth=1.0, zorder=0)


def _save(fig, out_dir: str | None, name: str) -> None:
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, name + ".png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print("Saved", path)


def _overall_deficit_df(df: pd.DataFrame) -> np.ndarray | None:
    """Per-tick overall deficit; same formula as plot_logged_run."""
    n = 100.0
    need = ["m0_energy", "m0_closeness_child", "m0_fear_threat", "m0_stress", "m0_fatigue", "m0_bonding"]
    if not all(c in df.columns for c in need):
        return None
    e = pd.to_numeric(df["m0_energy"], errors="coerce").to_numpy()
    close = pd.to_numeric(df["m0_closeness_child"], errors="coerce").to_numpy()
    fear = pd.to_numeric(df["m0_fear_threat"], errors="coerce").to_numpy()
    stress = pd.to_numeric(df["m0_stress"], errors="coerce").to_numpy()
    fatigue = pd.to_numeric(df["m0_fatigue"], errors="coerce").to_numpy()
    bond = pd.to_numeric(df["m0_bonding"], errors="coerce").to_numpy()
    m_energy_def = np.maximum(0, 80.0 - e) / n
    m_closeness_def = np.abs(close - 50.0) / n
    m_fear_def = np.maximum(0, fear - 0.0) / n
    m_stress_def = np.maximum(0, stress - 0.0) / n
    m_fatigue_def = np.maximum(0, fatigue - 0.0) / n
    m_bonding_def = np.maximum(0, 100.0 - bond) / n
    total = (
        m_energy_def
        + m_closeness_def
        + m_fear_def
        + m_stress_def
        + m_fatigue_def
        + m_bonding_def
    )
    if all(c in df.columns for c in ("c0_hunger", "c0_warmth", "c0_injury")):
        h = pd.to_numeric(df["c0_hunger"], errors="coerce").to_numpy()
        w = pd.to_numeric(df["c0_warmth"], errors="coerce").to_numpy()
        inj = pd.to_numeric(df["c0_injury"], errors="coerce").to_numpy()
        total = (
            total
            + np.maximum(0, h - 0.0) / n
            + np.abs(w - 50.0) * 2 / n
            + np.maximum(0, inj - 0.0) / n
        )
    return total


def _full_figure_title(run_label: str, title_note: str, survival_line: str) -> str:
    parts = []
    if run_label.strip():
        parts.append(run_label.strip())
    parts.append(title_note)
    if survival_line:
        parts.append(survival_line)
    return "\n".join(parts)


def plot_aggregate(
    dfs: list[pd.DataFrame],
    *,
    out_dir: str | None,
    ddof: int,
    mother_viz: str = "together",
    suptitle_suffix: str = "",
    run_label: str = "",
) -> None:
    if not dfs:
        print("No runs to plot.")
        return
    T = max(len(d) for d in dfs)
    x = np.arange(T, dtype=float)
    n_seeds = len(dfs)
    ref = dfs[0]

    moms = _agent_indices(ref, "m")
    kids = _agent_indices(ref, "c")
    if not moms and any(c.startswith("m0_") for c in ref.columns):
        moms = [0]
    if not kids and any(c.startswith("c0_") for c in ref.columns):
        kids = [0]

    mi0 = moms[0] if moms else 0
    ki0 = kids[0] if kids else 0
    m_en = f"m{mi0}_energy"
    c_hu = f"c{ki0}_hunger"
    survival_line = ""
    dm = dc = np.array([])
    sx = s_frac_m = s_frac_c = x
    short_note = ""
    if m_en in ref.columns and c_hu in ref.columns:
        survival_line, short_note, sx, s_frac_m, s_frac_c, dm, dc = _survival_summary_lines(
            dfs, T=T, mi=mi0, ki=ki0
        )

    colors_m = plt.cm.tab10(np.linspace(0, 1, max(len(moms), 1)))
    colors_c = plt.cm.Set2(np.linspace(0, 1, max(len(kids), 1)))
    mot_names = ["Forage", "Care", "Self", "Protect"]
    mot_colors = {"Forage": "goldenrod", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}

    title_note = f"mean ± std across {n_seeds} seeds{(' — ' + suptitle_suffix) if suptitle_suffix else ''} (variance ddof={ddof})"
    big_title = _full_figure_title(run_label, title_note, survival_line)

    # ----- Survival (fraction of runs still alive) -----
    if survival_line and len(dm):
        fig_s, ax_s = plt.subplots(figsize=(8, 3.0))
        ax_s.plot(sx, s_frac_m, color="steelblue", linewidth=1.6, label="Mother alive (fraction of runs)")
        ax_s.plot(sx, s_frac_c, color="indianred", linewidth=1.6, label="Child alive (fraction of runs)")
        ax_s.set_ylim(-0.02, 1.02)
        ax_s.set_ylabel("Fraction alive")
        _apply_xlabel_time_step(ax_s)
        ax_s.grid(True, alpha=0.3)
        h0, _ = ax_s.get_legend_handles_labels()
        h0.extend(
            [
                mlines.Line2D([], [], color="steelblue", linestyle="--", label="Mean mother death tick"),
                mlines.Line2D([], [], color="indianred", linestyle="--", label="Mean child death tick"),
            ]
        )
        ax_s.legend(handles=h0, loc="lower left", fontsize=8)
        _mark_mean_death_ticks(ax_s, dm, dc)
        t_surv = _full_figure_title(
            run_label,
            f"Survival over time — {short_note}",
            survival_line,
        )
        fig_s.suptitle(t_surv, fontsize=9, y=1.02)
        fig_s.tight_layout()
        _save(fig_s, out_dir, "survival_agg")
        plt.close(fig_s)

    # ----- Mother states -----
    want_m = list(ALL_MOTHER_STATES)
    if moms and mother_viz == "together":
        state_cols = [s for s in want_m if any(f"m{i}_{s}" in ref.columns for i in moms)]
        if state_cols:
            n = len(state_cols)
            ncol = min(2, n)
            nrow = max(1, (n + ncol - 1) // ncol)
            fig1, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 3 * nrow), sharex=True)
            axes = np.atleast_1d(axes).flatten()
            for ax, state_name in zip(axes, state_cols):
                ax.set_title(state_name)
                ax.set_ylabel("Value")
                ax.set_ylim(0, 100)
                ax.grid(True, alpha=0.3)
                for i, mi in enumerate(moms):
                    col = f"m{mi}_{state_name}"
                    if col not in ref.columns:
                        continue
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    _band(ax, x, mean, var, colors_m[i % len(colors_m)], f"M{mi}", ddof)
                ax.legend(loc="upper right", fontsize=7)
                if len(dm):
                    _mark_mean_death_ticks(ax, dm, dc)
            for j in range(len(state_cols), len(axes)):
                axes[j].set_visible(False)
            bi = min(len(state_cols) - 1, len(axes) - 1)
            _apply_xlabel_time_step(axes[bi])
            fig1.suptitle(f"Mother states\n{big_title}", fontsize=9, y=1.02)
            fig1.tight_layout(rect=[0, 0, 1, 0.95])
            _save(fig1, out_dir, "mother_states_agg")
            plt.close(fig1)

    # ----- Child states -----
    want_c = list(ALL_CHILD_STATES)
    if kids:
        state_cols = [s for s in want_c if any(f"c{i}_{s}" in ref.columns for i in kids)]
        if state_cols:
            fig2, axes = plt.subplots(1, len(state_cols), figsize=(3 * len(state_cols), 2.8), sharex=True)
            if len(state_cols) == 1:
                axes = [axes]
            for ax, state_name in zip(axes, state_cols):
                ax.set_title(state_name)
                ax.set_ylabel("Value")
                ax.set_ylim(0, 100)
                ax.grid(True, alpha=0.3)
                for i, ki in enumerate(kids):
                    col = f"c{ki}_{state_name}"
                    if col not in ref.columns:
                        continue
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    _band(ax, x, mean, var, colors_c[i % len(colors_c)], f"C{ki}", ddof)
                ax.legend(loc="upper right", fontsize=7)
                if len(dm):
                    _mark_mean_death_ticks(ax, dm, dc)
            _apply_xlabel_time_step(axes[-1])
            fig2.suptitle(f"Child states\n{big_title}", fontsize=9, y=1.02)
            fig2.tight_layout(rect=[0, 0, 1, 0.95])
            _save(fig2, out_dir, "child_states_agg")
            plt.close(fig2)

    # ----- Motivation values -----
    if moms:
        if len(moms) > 1 and mother_viz == "together":
            fig3, axes = plt.subplots(2, 2, figsize=(8, 4.5), sharex=True)
            axes = axes.flatten()
            for ai, mot in enumerate(mot_names):
                ax = axes[ai]
                ax.set_title(mot)
                ax.set_ylabel("Value")
                ax.set_ylim(0, 100)
                ax.grid(True, alpha=0.3)
                for i, mi in enumerate(moms):
                    col = f"m{mi}_mot_{mot}"
                    if col not in ref.columns:
                        continue
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    _band(ax, x, mean, var, colors_m[i % len(colors_m)], f"M{mi}", ddof)
                ax.legend(loc="upper right", fontsize=7)
                if len(dm):
                    _mark_mean_death_ticks(ax, dm, dc)
            _apply_xlabel_time_step(axes[-1])
            fig3.suptitle(f"Motivation values\n{big_title}", fontsize=9, y=1.02)
            fig3.tight_layout(rect=[0, 0, 1, 0.95])
            _save(fig3, out_dir, "motivation_values_agg")
            plt.close(fig3)
        else:
            for mi in moms if mother_viz == "separate" else [moms[0]]:
                mot_cols = [f"m{mi}_mot_{mot}" for mot in mot_names if f"m{mi}_mot_{mot}" in ref.columns]
                if not mot_cols:
                    continue
                fig3, ax = plt.subplots(figsize=(7, 3.2))
                for col in mot_cols:
                    mot = col.replace(f"m{mi}_mot_", "")
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    _band(ax, x, mean, var, mot_colors.get(mot, "gray"), mot, ddof)
                ax.set_title(f"Motivation values M{mi}" if len(moms) > 1 else "Motivation values")
                ax.set_ylabel("Value")
                _apply_xlabel_time_step(ax)
                ax.set_ylim(0, 100)
                ax.legend(loc="upper right", ncol=4)
                ax.grid(True, alpha=0.3)
                if len(dm):
                    _mark_mean_death_ticks(ax, dm, dc)
                fig3.suptitle(big_title, fontsize=9, y=1.02)
                fig3.tight_layout(rect=[0, 0, 1, 0.94])
                _save(fig3, out_dir, f"motivation_values_agg_m{mi}" if len(moms) > 1 else "motivation_values_agg")
                plt.close(fig3)

    # ----- Selected motivation -----
    if moms:
        if len(moms) > 1 and mother_viz == "together":
            fig4, ax = plt.subplots(figsize=(8, 2.8))
            for i, mi in enumerate(moms):
                for mot in mot_names:
                    col = f"m{mi}_sel_{mot}"
                    if col not in ref.columns:
                        continue
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    lbl = f"M{mi} {mot}"
                    _band(ax, x, mean, var, colors_m[i % len(colors_m)], lbl, ddof)
            ax.set_title("Selected motivation")
            ax.set_ylabel("Mean activity")
            _apply_xlabel_time_step(ax)
            ax.set_ylim(-0.05, 1.05)
            ax.legend(loc="upper right", ncol=2, fontsize=6)
            ax.grid(True, alpha=0.3)
            if len(dm):
                _mark_mean_death_ticks(ax, dm, dc)
            fig4.suptitle(big_title, fontsize=9, y=1.02)
            fig4.tight_layout(rect=[0, 0, 1, 0.94])
            _save(fig4, out_dir, "selected_motivation_agg")
            plt.close(fig4)
        else:
            for mi in moms if mother_viz == "separate" else [moms[0]]:
                sel_cols = [f"m{mi}_sel_{mot}" for mot in mot_names if f"m{mi}_sel_{mot}" in ref.columns]
                if not sel_cols:
                    continue
                fig4, ax = plt.subplots(figsize=(7, 2.6))
                for col in sel_cols:
                    mot = col.replace(f"m{mi}_sel_", "")
                    mat = _stack_column(dfs, col, T)
                    mean, var = _nanmean_nanvar(mat, ddof=ddof)
                    _band(ax, x, mean, var, mot_colors.get(mot, "gray"), mot, ddof)
                ax.set_title(f"Selected motivation M{mi}" if len(moms) > 1 else "Selected motivation (0/1)")
                ax.set_ylabel("Mean activity")
                _apply_xlabel_time_step(ax)
                ax.set_ylim(-0.05, 1.05)
                ax.legend(loc="upper right", ncol=4)
                ax.grid(True, alpha=0.3)
                if len(dm):
                    _mark_mean_death_ticks(ax, dm, dc)
                fig4.suptitle(big_title, fontsize=9, y=1.02)
                fig4.tight_layout(rect=[0, 0, 1, 0.94])
                _save(fig4, out_dir, f"selected_motivation_agg_m{mi}" if len(moms) > 1 else "selected_motivation_agg")
                plt.close(fig4)

    # ----- Overall deficit -----
    deficits = []
    for df in dfs:
        s = _overall_deficit_df(df)
        if s is None:
            deficits = []
            break
        L = len(s)
        padded = np.full(T, np.nan)
        padded[:L] = s[: min(L, T)]
        deficits.append(padded)
    if deficits:
        mat = np.vstack(deficits)
        mean, var = _nanmean_nanvar(mat, ddof=ddof)
        fig7, ax = plt.subplots(figsize=(8, 2.6))
        _band(ax, x, mean, var, "black", "overall deficit", ddof)
        ax.set_title("Overall deficit")
        ax.set_ylabel("Deficit")
        _apply_xlabel_time_step(ax)
        ax.grid(True, alpha=0.3)
        ax.legend()
        if len(dm):
            _mark_mean_death_ticks(ax, dm, dc)
        fig7.suptitle(big_title, fontsize=9, y=1.02)
        fig7.tight_layout(rect=[0, 0, 1, 0.94])
        _save(fig7, out_dir, "overall_deficit_agg")
        plt.close(fig7)


def _parse_args():
    p = argparse.ArgumentParser(description="N rollouts + aggregate mean/variance plots.")
    p.add_argument("--genome", type=str, required=True, help="Motivation genome JSON.")
    p.add_argument("--n-seeds", type=int, default=16, dest="n_seeds")
    p.add_argument("--seed-base", type=int, default=10_000, dest="seed_base")
    p.add_argument("--max-ticks", type=int, default=DEFAULT_MAX_TICKS)
    p.add_argument("--ddof", type=int, default=0, help="ddof for numpy.nanvar across seeds (0=population).")
    p.add_argument("--out", "-o", type=str, default="figures_agg", help="PNG output directory.")
    p.add_argument("--stop-when-both-dead", action="store_true")
    p.add_argument("--grid-w", type=int, default=DEFAULT_GRID_W)
    p.add_argument("--grid-h", type=int, default=DEFAULT_GRID_H)
    p.add_argument("--mothers", type=int, default=DEFAULT_NUM_MOTHERS)
    p.add_argument("--children", type=int, default=DEFAULT_NUM_CHILDREN)
    p.add_argument("--threats", type=int, default=DEFAULT_NUM_THREATS)
    p.add_argument("--food-start", type=int, default=DEFAULT_FOOD_AT_START)
    p.add_argument("--food-spawn-interval", type=int, default=DEFAULT_FOOD_SPAWN_INTERVAL)
    p.add_argument("--food-spawn-n", type=int, default=DEFAULT_FOOD_SPAWN_N)
    p.add_argument("--day-step", type=int, default=DEFAULT_DAY_STEP)
    # Plasticity (same knobs as headless_rollout_log / baseline)
    p.add_argument(
        "--plasticity",
        choices=["none", "outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"],
        default="none",
        help="Lifetime plasticity rule during each rollout (none = frozen u_plastic).",
    )
    p.add_argument(
        "--deficit-signal",
        choices=["global", "local"],
        default="global",
        help="Plasticity deficit signal: global=overall; local=motivation-aligned.",
    )
    p.add_argument(
        "--learn-w",
        choices=["on", "off"],
        default="off",
        help="Whether psych weights w update (off = only motivation u).",
    )
    p.add_argument(
        "--update-mode",
        choices=["per_tick", "segment", "segment_capped"],
        default="per_tick",
        help="Plasticity credit assignment timing.",
    )
    p.add_argument(
        "--segment-kmax",
        type=int,
        default=20,
        help="For segment_capped: max ticks per segment before forced update.",
    )
    p.add_argument(
        "--save-csv-dir",
        type=str,
        default=None,
        help="If set, write one CSV per seed into this directory.",
    )
    p.add_argument(
        "--label",
        type=str,
        default="",
        help="Short label (e.g. genome name) prepended to every figure title.",
    )
    return p.parse_args()


def main():
    args = _parse_args()
    gpath = os.path.abspath(args.genome)
    if not os.path.isfile(gpath):
        print("Genome not found:", gpath, file=sys.stderr)
        sys.exit(1)
    with open(gpath, encoding="utf-8") as f:
        genome = json.load(f)
    if not isinstance(genome, dict):
        print("Genome JSON must be a dict.", file=sys.stderr)
        sys.exit(1)

    rule = None if str(args.plasticity) == "none" else str(args.plasticity)
    cfg = {
        "grid_w": args.grid_w,
        "grid_h": args.grid_h,
        "num_mothers": args.mothers,
        "num_children": args.children,
        "num_threats": args.threats,
        "food_at_start": args.food_start,
        "food_spawn_interval": args.food_spawn_interval,
        "food_spawn_n": args.food_spawn_n,
        "day_step": args.day_step,
        "plasticity_rule": rule,
        "plasticity_deficit_signal": str(args.deficit_signal),
        "plasticity_learn_w": (str(args.learn_w) == "on"),
        "plasticity_update_mode": str(args.update_mode),
        "plasticity_segment_kmax": int(args.segment_kmax),
    }

    n = int(args.n_seeds)
    if n < 2:
        print("Use --n-seeds >= 2 for meaningful variance bands.", file=sys.stderr)

    dfs: list[pd.DataFrame] = []
    if args.save_csv_dir:
        os.makedirs(args.save_csv_dir, exist_ok=True)

    for i in range(n):
        seed = int(args.seed_base) + i
        world = _world_from_genome_and_seed(seed, genome, cfg)
        df = collect_rollout_dataframe(
            world,
            int(args.max_ticks),
            stop_when_both_dead=bool(args.stop_when_both_dead),
        )
        dfs.append(df)
        if args.save_csv_dir:
            path = os.path.join(args.save_csv_dir, f"run_seed_{seed}.csv")
            df.to_csv(path, index=False)

    plot_aggregate(
        dfs,
        out_dir=args.out,
        ddof=int(args.ddof),
        mother_viz="together",
        run_label=str(args.label or ""),
    )
    print("Done.")


if __name__ == "__main__":
    main()
