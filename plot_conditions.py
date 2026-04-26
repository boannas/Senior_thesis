"""
Plot survival + state trajectories from baseline_passive_lower_bound.py outputs.

Expected per-condition folder structure (produced by baseline_passive_lower_bound.py):
- survival_experiment_replicates.csv
- passive_lower_bound_timeseries_repXXX_seedYYYY.csv   (one per replicate)

Usage examples:
  python plot_conditions.py --root test_results
  python plot_conditions.py --dirs test_results/cond_passive_no_threat test_results/cond_active_threat
"""

from __future__ import annotations

import argparse
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import pandas as pd
except ImportError as e:
    raise SystemExit("pandas required. Install: pip install pandas") from e

try:
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit("matplotlib required. Install: pip install matplotlib") from e


@dataclass
class ConditionData:
    name: str
    path: str
    replicates_path: str
    timeseries_paths: List[str]


def _parse_factors(condition_name: str) -> Dict[str, str]:
    """
    Best-effort parsing of factors from folder name.
    Expected names like:
      passive_no_threat, passive_threat, active_no_threat, active_threat
      cond_passive_no_threat, etc.
    """
    n = condition_name.lower()
    mode = "active" if "active" in n else "passive" if "passive" in n else "unknown"
    threat = "no_threat" if ("no_threat" in n or "no-threat" in n) else "threat" if "threat" in n else "unknown"
    return {"mode": mode, "threat": threat}


def _km_curve(time: np.ndarray, event: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Kaplan–Meier estimate. Returns step points (t, S(t))."""
    time = np.asarray(time, dtype=float)
    event = np.asarray(event, dtype=int)
    order = np.argsort(time)
    time = time[order]
    event = event[order]
    uniq_times = np.unique(time)
    at_risk = len(time)
    s = 1.0
    t_out = [0.0]
    s_out = [1.0]
    for t in uniq_times:
        mask = (time == t)
        d = int(np.sum(event[mask] == 1))
        c = int(np.sum(event[mask] == 0))
        if at_risk > 0 and d > 0:
            s *= (1.0 - d / at_risk)
        t_out.append(float(t))
        s_out.append(float(s))
        at_risk -= (d + c)
    return np.array(t_out, dtype=float), np.array(s_out, dtype=float)


def _logrank_test(time_a: np.ndarray, event_a: np.ndarray, time_b: np.ndarray, event_b: np.ndarray) -> Tuple[float, float]:
    """
    Two-sample log-rank test (df=1). Returns (chi2, p).
    p-value uses chi-square survival for df=1: p = erfc(sqrt(chi2/2)).
    """
    time_a = np.asarray(time_a, dtype=float)
    event_a = np.asarray(event_a, dtype=int)
    time_b = np.asarray(time_b, dtype=float)
    event_b = np.asarray(event_b, dtype=int)
    times = np.unique(np.concatenate([time_a[event_a == 1], time_b[event_b == 1]]))
    if times.size == 0:
        return float("nan"), float("nan")
    Oa = Ea = Va = 0.0
    for t in times:
        ra = np.sum(time_a >= t)
        rb = np.sum(time_b >= t)
        r = ra + rb
        if r <= 1:
            continue
        da = np.sum((time_a == t) & (event_a == 1))
        db = np.sum((time_b == t) & (event_b == 1))
        d = da + db
        if d <= 0:
            continue
        exp_a = d * (ra / r)
        var_a = (ra * rb * d * (r - d)) / (r * r * (r - 1))
        Oa += da
        Ea += exp_a
        Va += var_a
    if Va <= 0:
        return float("nan"), float("nan")
    chi2 = (Oa - Ea) ** 2 / Va
    p = math.erfc(math.sqrt(chi2 / 2.0))
    return float(chi2), float(p)


def _hazard_curve(time: np.ndarray, event: np.ndarray, max_t: int) -> Tuple[np.ndarray, np.ndarray]:
    """Discrete hazard per tick: deaths(t)/at_risk(t)."""
    time = np.asarray(time, dtype=int)
    event = np.asarray(event, dtype=int)
    hz = np.full(max_t + 1, np.nan, dtype=float)
    for t in range(max_t + 1):
        at_risk = np.sum(time >= t)
        if at_risk <= 0:
            break
        deaths = np.sum((time == t) & (event == 1))
        hz[t] = deaths / at_risk
    return np.arange(max_t + 1, dtype=float), hz


def _discover_conditions(root: str) -> List[ConditionData]:
    out: List[ConditionData] = []
    for dirpath, _dirnames, filenames in os.walk(root):
        if "survival_experiment_replicates.csv" not in filenames:
            continue
        rep_path = os.path.join(dirpath, "survival_experiment_replicates.csv")
        ts_paths = sorted(
            glob.glob(os.path.join(dirpath, "passive_lower_bound_timeseries_rep*_seed*.csv"))
        )
        name = os.path.relpath(dirpath, root)
        out.append(
            ConditionData(
                name=name,
                path=dirpath,
                replicates_path=rep_path,
                timeseries_paths=ts_paths,
            )
        )
    return sorted(out, key=lambda c: c.name)


def _load_timeseries(paths: List[str]) -> pd.DataFrame:
    if not paths:
        return pd.DataFrame()
    frames = []
    for p in paths:
        df = pd.read_csv(p)
        frames.append(df)
    df_all = pd.concat(frames, ignore_index=True)
    # Ensure numeric columns
    for c in df_all.columns:
        if c in ("mother_selected_motivation",):
            continue
        df_all[c] = pd.to_numeric(df_all[c], errors="coerce")
    return df_all


def _mean_ci_band(x: np.ndarray, y: np.ndarray, z: float = 1.96) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    y is shape (n,). Return mean and +/- z*sem.
    """
    mean = np.nanmean(y)
    std = np.nanstd(y)
    n = np.sum(~np.isnan(y))
    sem = (std / np.sqrt(n)) if n > 0 else np.nan
    return mean, mean - z * sem, mean + z * sem


def _aggregate_over_time(
    df_all: pd.DataFrame, value_col: str, tick_col: str = "tick"
) -> pd.DataFrame:
    """
    Returns a DataFrame with columns: tick, day, mean, lo, hi.
    CI uses normal approx (z=1.96).
    """
    if df_all.empty or value_col not in df_all.columns:
        return pd.DataFrame()

    grouped = df_all.groupby(tick_col, sort=True)
    ticks = []
    days = []
    means = []
    los = []
    his = []
    for tick, g in grouped:
        y = g[value_col].to_numpy(dtype=float)
        m, lo, hi = _mean_ci_band(np.array([tick], dtype=float), y)
        ticks.append(float(tick))
        # day column exists in the log; take mean
        if "day" in g.columns:
            days.append(float(np.nanmean(g["day"].to_numpy(dtype=float))))
        else:
            days.append(float(tick))
        means.append(float(m))
        los.append(float(lo))
        his.append(float(hi))
    return pd.DataFrame({"tick": ticks, "day": days, "mean": means, "lo": los, "hi": his})


def _aggregate_survival(df_all: pd.DataFrame, alive_col: str) -> pd.DataFrame:
    """
    alive fraction over time.
    """
    if df_all.empty or alive_col not in df_all.columns:
        return pd.DataFrame()
    grouped = df_all.groupby("tick", sort=True)
    rows = []
    for tick, g in grouped:
        alive = g[alive_col].to_numpy(dtype=float)
        frac = float(np.nanmean(alive)) if alive.size else np.nan
        day = float(np.nanmean(g["day"])) if "day" in g.columns else float(tick)
        rows.append({"tick": float(tick), "day": day, "alive_frac": frac})
    return pd.DataFrame(rows)


def _plot_band(ax, x, mean, lo, hi, label: str, color: str):
    ax.plot(x, mean, label=label, color=color, linewidth=2)
    ax.fill_between(x, lo, hi, color=color, alpha=0.2, linewidth=0)


def plot_condition(cond: ConditionData, *, use_days: bool = True) -> None:
    df_all = _load_timeseries(cond.timeseries_paths)
    if df_all.empty:
        print(f"[WARN] No timeseries CSVs found for {cond.name} at {cond.path}")
        return

    xcol = "day" if use_days else "tick"

    # --- 1) Survival curves (child + mother alive fraction) ---
    s_child = _aggregate_survival(df_all, "child_alive")
    s_mother = _aggregate_survival(df_all, "mother_alive")

    # Auto-zoom window for child-focused plots:
    # show data until child population is extinct (alive_frac hits 0), plus a small margin.
    child_end_x = None
    if not s_child.empty and xcol in s_child.columns:
        alive = s_child["alive_frac"].to_numpy(dtype=float)
        x = s_child[xcol].to_numpy(dtype=float)
        alive_mask = np.isfinite(alive) & (alive > 0.0)
        if np.any(alive_mask):
            child_end_x = float(np.nanmax(x[alive_mask]))
    child_margin = 0.5 if use_days else 50.0  # 0.5 day (12h) or 50 ticks
    child_xlim = None
    if child_end_x is not None:
        child_xlim = (0.0, child_end_x + child_margin)

    fig, ax = plt.subplots(figsize=(7, 3))
    if not s_child.empty:
        ax.plot(s_child[xcol], s_child["alive_frac"], label="Child alive fraction", color="tab:blue", linewidth=2)
    if not s_mother.empty:
        ax.plot(s_mother[xcol], s_mother["alive_frac"], label="Mother alive fraction", color="tab:red", linewidth=2)
    ax.set_title(f"Survival — {cond.name}")
    ax.set_xlabel("Days" if use_days else "Tick")
    ax.set_ylabel("Alive fraction")
    ax.set_ylim(-0.02, 1.02)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(cond.path, "plot_survival.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)

    # --- 2) Child states ---
    child_cols = [
        ("child_hunger", "Hunger", "tab:orange"),
        ("child_warmth", "Warmth", "tab:cyan"),
        ("child_injury", "Injury", "tab:purple"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10, 3), sharex=True)
    for ax, (col, title, color) in zip(axes, child_cols):
        agg = _aggregate_over_time(df_all, col)
        if agg.empty:
            ax.set_visible(False)
            continue
        _plot_band(ax, agg[xcol], agg["mean"], agg["lo"], agg["hi"], label=title, color=color)
        ax.set_title(title)
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.3)
        if child_xlim is not None:
            ax.set_xlim(*child_xlim)
    axes[0].set_ylabel("Value")
    for ax in axes:
        ax.set_xlabel("Days" if use_days else "Tick")
    fig.suptitle(f"Child states (mean ± 95% CI) — {cond.name}")
    fig.tight_layout()
    fig.savefig(os.path.join(cond.path, "plot_child_states.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)

    # --- 3) Mother physiology ---
    mother_cols = [
        ("mother_energy", "Energy", "tab:red"),
        ("mother_fatigue", "Fatigue", "tab:green"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(7.5, 3), sharex=True)
    for ax, (col, title, color) in zip(axes, mother_cols):
        agg = _aggregate_over_time(df_all, col)
        if agg.empty:
            ax.set_visible(False)
            continue
        _plot_band(ax, agg[xcol], agg["mean"], agg["lo"], agg["hi"], label=title, color=color)
        ax.set_title(title)
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("Value")
    for ax in axes:
        ax.set_xlabel("Days" if use_days else "Tick")
    fig.suptitle(f"Mother physiology (mean ± 95% CI) — {cond.name}")
    fig.tight_layout()
    fig.savefig(os.path.join(cond.path, "plot_mother_states.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)

    # --- 3b) Plasticity diagnostics (if present) ---
    diag_cols = [
        ("deficit_before", "Overall deficit (before)", "tab:gray", (0, None)),
        ("learning_rate_eff", "Effective learning rate", "tab:olive", (0, None)),
        ("u_drift", "Motivation weight drift |u_p-u_f|", "tab:blue", (0, None)),
        ("w_drift", "Psych weight drift |w_p-w_f|", "tab:purple", (0, None)),
    ]
    present = [c for c, *_ in diag_cols if c in df_all.columns]
    if present:
        fig, axes = plt.subplots(len(diag_cols), 1, figsize=(7.5, 2.2 * len(diag_cols)), sharex=True)
        if len(diag_cols) == 1:
            axes = [axes]
        for ax, (col, title, color, _ylim) in zip(axes, diag_cols):
            if col not in df_all.columns:
                ax.set_visible(False)
                continue
            agg = _aggregate_over_time(df_all, col)
            if agg.empty:
                ax.set_visible(False)
                continue
            ax.plot(agg[xcol], agg["mean"], color=color, linewidth=2)
            ax.fill_between(agg[xcol], agg["lo"], agg["hi"], color=color, alpha=0.2, linewidth=0)
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Days" if use_days else "Tick")
        fig.suptitle(f"Plasticity diagnostics (mean ± 95% CI) — {cond.name}")
        fig.tight_layout()
        fig.savefig(os.path.join(cond.path, "plot_plasticity_diagnostics.png"), dpi=160, bbox_inches="tight")
        plt.close(fig)

    # --- 3c) Key weight traces (fixed vs plastic) ---
    key_pairs = [
        ("u_forage_child_hunger_fixed", "u_forage_child_hunger_plastic", "u forage: child_hunger"),
        ("u_care_child_warmth_fixed", "u_care_child_warmth_plastic", "u care: child_warmth"),
        ("w_fear_threat_gain_fixed", "w_fear_threat_gain_plastic", "w fear: threat_gain"),
        ("w_fear_decay_fixed", "w_fear_decay_plastic", "w fear: decay"),
        ("w_oxytocin_closeness_gain_fixed", "w_oxytocin_closeness_gain_plastic", "w oxytocin: closeness_gain"),
        ("w_oxytocin_decay_fixed", "w_oxytocin_decay_plastic", "w oxytocin: decay"),
    ]
    any_key = any((a in df_all.columns and b in df_all.columns) for a, b, _ in key_pairs)
    if any_key:
        fig, axes = plt.subplots(3, 2, figsize=(10, 7), sharex=True)
        axes = axes.flatten()
        for ax, (fixed_col, plastic_col, title) in zip(axes, key_pairs):
            if fixed_col not in df_all.columns or plastic_col not in df_all.columns:
                ax.set_visible(False)
                continue
            f_agg = _aggregate_over_time(df_all, fixed_col)
            p_agg = _aggregate_over_time(df_all, plastic_col)
            if f_agg.empty or p_agg.empty:
                ax.set_visible(False)
                continue
            ax.plot(f_agg[xcol], f_agg["mean"], linestyle="--", color="black", linewidth=1.5, label="fixed")
            ax.plot(p_agg[xcol], p_agg["mean"], linestyle="-", color="tab:orange", linewidth=2, label="plastic")
            ax.set_title(title)
            ax.set_ylim(-0.05, 2.05)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7, loc="upper right")
        for ax in axes[-2:]:
            ax.set_xlabel("Days" if use_days else "Tick")
        fig.suptitle(f"Key weights: fixed vs plastic (mean) — {cond.name}")
        fig.tight_layout()
        fig.savefig(os.path.join(cond.path, "plot_key_weights.png"), dpi=160, bbox_inches="tight")
        plt.close(fig)

    # --- 4) Threat distance + close radius (if present) ---
    if "threat_min_dist_to_child" in df_all.columns:
        fig, ax = plt.subplots(figsize=(7, 3))
        dist = _aggregate_over_time(df_all, "threat_min_dist_to_child")
        rad = _aggregate_over_time(df_all, "child_threat_close_radius")
        if not dist.empty:
            ax.plot(dist[xcol], dist["mean"], color="tab:gray", linewidth=2, label="min threat distance")
        if not rad.empty:
            ax.plot(rad[xcol], rad["mean"], color="tab:pink", linewidth=2, label="recovery radius")
        ax.set_title(f"Threat distance vs recovery radius — {cond.name}")
        ax.set_xlabel("Days" if use_days else "Tick")
        ax.set_ylabel("Cells (octile distance)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(os.path.join(cond.path, "plot_threat_distance.png"), dpi=160, bbox_inches="tight")
        plt.close(fig)

    print(f"Saved plots in: {cond.path}")


def main():
    p = argparse.ArgumentParser(description="Plot condition folders from test_results logs.")
    p.add_argument("--root", default=None, help="Root directory to scan for condition folders.")
    p.add_argument("--dirs", nargs="*", default=None, help="Explicit condition directories to plot.")
    p.add_argument("--x", choices=["day", "tick"], default="day", help="X-axis unit.")
    p.add_argument("--compare", action="store_true", default=True,
                   help="Also write a cross-condition comparison figure (default: on).")
    p.add_argument("--no-compare", action="store_false", dest="compare",
                   help="Disable cross-condition comparison figure.")
    args = p.parse_args()

    if args.dirs:
        conds = []
        for d in args.dirs:
            rep = os.path.join(d, "survival_experiment_replicates.csv")
            if not os.path.isfile(rep):
                raise SystemExit(f"Missing survival_experiment_replicates.csv in {d}")
            ts = sorted(glob.glob(os.path.join(d, "passive_lower_bound_timeseries_rep*_seed*.csv")))
            conds.append(ConditionData(name=os.path.basename(d.rstrip("/\\")), path=d, replicates_path=rep, timeseries_paths=ts))
    else:
        if not args.root:
            raise SystemExit("Provide --root or --dirs.")
        conds = _discover_conditions(args.root)

    if not conds:
        raise SystemExit("No condition folders found.")

    use_days = (args.x == "day")
    for cond in conds:
        plot_condition(cond, use_days=use_days)

    if args.compare:
        out_dir = args.root if args.root else os.path.dirname(os.path.abspath(conds[0].path))
        day_step = 100.0

        # Load replicate-level tables for each condition
        rep_by_cond: Dict[str, pd.DataFrame] = {}
        for cond in conds:
            if os.path.isfile(cond.replicates_path):
                rep_by_cond[cond.name] = pd.read_csv(cond.replicates_path)

        cond_names = sorted(rep_by_cond.keys())
        if not cond_names:
            return

        # ----------------------------
        # (1) Kaplan–Meier (split)
        # ----------------------------
        def _style_for(name: str):
            f = _parse_factors(name)
            ls = "-" if f["mode"] == "active" else "--"
            if f["threat"] == "threat":
                c = "tab:red"
            elif f["threat"] == "no_threat":
                c = "tab:blue"
            else:
                c = "tab:gray"
            return ls, c

        def _plot_km(agent: str, out_name: str):
            fig, ax = plt.subplots(figsize=(7.5, 4))
            for name in cond_names:
                df = rep_by_cond[name]
                if df.empty:
                    continue
                max_ticks = float(np.nanmax(pd.to_numeric(df["max_ticks"], errors="coerce")))
                t = pd.to_numeric(df[f"{agent}_death_tick"], errors="coerce").to_numpy(dtype=float)
                t = t[np.isfinite(t)]
                if t.size == 0:
                    continue
                e = (t < max_ticks).astype(int)
                tt, ss = _km_curve(t / day_step, e)
                ls, c = _style_for(name)
                ax.step(tt, ss, where="post", linestyle=ls, color=c, linewidth=2, label=name)
            ax.set_title(f"Kaplan–Meier survival — {agent}")
            ax.set_xlabel("Days")
            ax.set_ylabel("Survival probability")
            ax.set_ylim(-0.02, 1.02)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7, loc="upper right")
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, out_name), dpi=180, bbox_inches="tight")
            plt.close(fig)

        _plot_km("child", "plot_km_child.png")
        _plot_km("mother", "plot_km_mother.png")

        # ----------------------------
        # (2) Boxplots (median/IQR)
        # ----------------------------
        child_days = []
        mother_days = []
        for name in cond_names:
            df = rep_by_cond[name]
            c = pd.to_numeric(df["child_death_tick"], errors="coerce").to_numpy(dtype=float) / day_step
            m = pd.to_numeric(df["mother_death_tick"], errors="coerce").to_numpy(dtype=float) / day_step
            child_days.append(c[np.isfinite(c)])
            mother_days.append(m[np.isfinite(m)])

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        axes[0].boxplot(child_days, labels=cond_names, showfliers=False)
        axes[0].set_title("Child time-to-death (median/IQR)")
        axes[0].set_ylabel("Days")
        axes[0].grid(True, axis="y", alpha=0.3)
        axes[0].tick_params(axis="x", rotation=25)

        axes[1].boxplot(mother_days, labels=cond_names, showfliers=False)
        axes[1].set_title("Mother time-to-death (median/IQR)")
        axes[1].set_ylabel("Days")
        axes[1].grid(True, axis="y", alpha=0.3)
        axes[1].tick_params(axis="x", rotation=25)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "plot_ttd_boxplots.png"), dpi=180, bbox_inches="tight")
        plt.close(fig)

        # ----------------------------
        # (3) Cumulative deaths + hazard
        # ----------------------------
        def _plot_hazard_and_cum(agent: str):
            fig, axes = plt.subplots(1, 2, figsize=(14, 4))
            ax_cum, ax_hz = axes
            for name in cond_names:
                df = rep_by_cond[name]
                if df.empty:
                    continue
                max_ticks = int(np.nanmax(pd.to_numeric(df["max_ticks"], errors="coerce")))
                t = pd.to_numeric(df[f"{agent}_death_tick"], errors="coerce").to_numpy(dtype=float)
                t = t[np.isfinite(t)]
                if t.size == 0:
                    continue
                e = (t < max_ticks).astype(int)
                tt, ss = _km_curve(t, e)
                ls, c = _style_for(name)
                ax_cum.step(tt / day_step, 1.0 - ss, where="post", linestyle=ls, color=c, linewidth=2, label=name)
                ht, hz = _hazard_curve(t.astype(int), e.astype(int), max_t=max_ticks)
                win = 5
                kernel = np.ones(win) / win
                hz_s = np.convolve(np.nan_to_num(hz, nan=0.0), kernel, mode="same")
                ax_hz.plot(ht / day_step, hz_s, linestyle=ls, color=c, linewidth=2, label=name)

            ax_cum.set_title(f"Cumulative deaths — {agent}")
            ax_cum.set_xlabel("Days")
            ax_cum.set_ylabel("Cumulative death fraction")
            ax_cum.set_ylim(-0.02, 1.02)
            ax_cum.grid(True, alpha=0.3)
            ax_cum.legend(fontsize=7, loc="lower right")

            ax_hz.set_title(f"Hazard rate (smoothed) — {agent}")
            ax_hz.set_xlabel("Days")
            ax_hz.set_ylabel("Hazard per tick")
            ax_hz.grid(True, alpha=0.3)
            ax_hz.legend(fontsize=7, loc="upper right")

            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, f"plot_hazard_cum_{agent}.png"), dpi=180, bbox_inches="tight")
            plt.close(fig)

        _plot_hazard_and_cum("child")
        _plot_hazard_and_cum("mother")

        # ----------------------------
        # (4) Log-rank tests (key comparisons)
        # ----------------------------
        stats_rows = []
        by_factor: Dict[Tuple[str, str], str] = {}
        for name in cond_names:
            f = _parse_factors(name)
            by_factor[(f["mode"], f["threat"])] = name

        def _add_lr(agent: str, a: str, b: str, label: str):
            dfa = rep_by_cond.get(a)
            dfb = rep_by_cond.get(b)
            if dfa is None or dfb is None or dfa.empty or dfb.empty:
                return
            max_a = float(np.nanmax(pd.to_numeric(dfa["max_ticks"], errors="coerce")))
            max_b = float(np.nanmax(pd.to_numeric(dfb["max_ticks"], errors="coerce")))
            max_ticks = min(max_a, max_b)
            ta = pd.to_numeric(dfa[f"{agent}_death_tick"], errors="coerce").to_numpy(dtype=float)
            tb = pd.to_numeric(dfb[f"{agent}_death_tick"], errors="coerce").to_numpy(dtype=float)
            ta = ta[np.isfinite(ta)]
            tb = tb[np.isfinite(tb)]
            ea = (ta < max_ticks).astype(int)
            eb = (tb < max_ticks).astype(int)
            chi2, p = _logrank_test(ta, ea, tb, eb)
            stats_rows.append({"comparison": label, "agent": agent, "cond_a": a, "cond_b": b, "chi2_df1": chi2, "p_value": p})

        for threat in ("no_threat", "threat"):
            a = by_factor.get(("active", threat))
            b = by_factor.get(("passive", threat))
            if a and b:
                _add_lr("child", a, b, f"active vs passive ({threat})")
                _add_lr("mother", a, b, f"active vs passive ({threat})")
        for mode in ("active", "passive"):
            a = by_factor.get((mode, "threat"))
            b = by_factor.get((mode, "no_threat"))
            if a and b:
                _add_lr("child", a, b, f"threat vs no_threat ({mode})")
                _add_lr("mother", a, b, f"threat vs no_threat ({mode})")

        if stats_rows:
            pd.DataFrame(stats_rows).to_csv(os.path.join(out_dir, "survival_stats_logrank.csv"), index=False)

        # ----------------------------
        # (5) Phase/regime plots (optional; uses timeseries)
        # ----------------------------
        phase_rows = []
        for cond in conds:
            if not cond.timeseries_paths:
                continue
            for pth in cond.timeseries_paths:
                df_ts = pd.read_csv(pth)
                if df_ts.empty or "child_alive" not in df_ts.columns:
                    continue
                alive = pd.to_numeric(df_ts["child_alive"], errors="coerce").fillna(0).to_numpy(dtype=float) > 0.0
                if not np.any(alive):
                    continue
                last = int(np.where(alive)[0].max())
                df_u = df_ts.iloc[: last + 1]
                surv_days = float(df_u["day"].iloc[-1]) if "day" in df_u.columns else float(df_u["tick"].iloc[-1]) / day_step
                def _m(col):
                    return float(pd.to_numeric(df_u[col], errors="coerce").mean()) if col in df_u.columns else np.nan
                phase_rows.append({
                    "condition": cond.name,
                    "replicate": int(df_u["replicate"].iloc[0]) if "replicate" in df_u.columns else -1,
                    "seed": int(df_u["seed"].iloc[0]) if "seed" in df_u.columns else -1,
                    "child_survival_days": surv_days,
                    "mean_child_hunger": _m("child_hunger"),
                    "mean_mother_energy": _m("mother_energy"),
                    "mean_mother_fear": _m("mother_fear_threat"),
                })

        if phase_rows:
            dfp = pd.DataFrame(phase_rows)
            dfp.to_csv(os.path.join(out_dir, "phase_regime_features.csv"), index=False)
            def _scatter(xcol, out_name, title):
                if xcol not in dfp.columns:
                    return
                fig, ax = plt.subplots(figsize=(6.5, 4))
                for cname in sorted(dfp["condition"].unique().tolist()):
                    sub = dfp[dfp["condition"] == cname]
                    ax.scatter(sub[xcol], sub["child_survival_days"], s=18, alpha=0.7, label=cname)
                ax.set_xlabel(xcol)
                ax.set_ylabel("child_survival_days")
                ax.set_title(title)
                ax.grid(True, alpha=0.3)
                ax.legend(fontsize=7, loc="best")
                fig.tight_layout()
                fig.savefig(os.path.join(out_dir, out_name), dpi=180, bbox_inches="tight")
                plt.close(fig)
            _scatter("mean_child_hunger", "plot_phase_survival_vs_hunger.png", "Survival vs mean child hunger")
            _scatter("mean_mother_energy", "plot_phase_survival_vs_mother_energy.png", "Survival vs mean mother energy")
            _scatter("mean_mother_fear", "plot_phase_survival_vs_mother_fear.png", "Survival vs mean mother fear")

        print(f"Saved comparison pack in: {out_dir}")


if __name__ == "__main__":
    main()

