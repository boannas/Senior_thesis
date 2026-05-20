#!/usr/bin/env python3
"""
plot_child_terminal_risk_profile.py
====================================
**Terminal risk profile** per mother **init type** (anti / random / pro),
optionally broken down by **evolve_seed** (genome).

What it measures (descriptive, not causal)
------------------------------------------
1. **Pre-death window** — last ``--window`` ticks where the child is still
   logged as alive. Child death = first row with non-finite ``c0_hunger``
   (same convention as ``plot_runlog_conditional_behavior.py`` TTD).

2. **Window means** — mean hunger, warmth, injury, mother fear_threat in
   that window. High terminal hunger / injury / low warmth = “child was in
   a bad part of state space right before logging stopped”.

3. **Threat–injury coupling (alive episode)** — Spearman rho between
   ``m0_fear_threat`` and ``c0_injury`` on all ticks where both are finite
   and the child is alive. Summarizes co-movement, not “penalty weights”.

Aggregation
-----------
Per ``run_*.csv`` → one row of metrics → mean within
``(plasticity, init, evolve_seed)`` across runs in that genome directory.

Outputs (under ``--out-dir``)
-----------------------------
  - ``child_terminal_profile_per_genome.csv`` (includes ``death_pathway_proxy``)
  - ``terminal_means_by_init.png`` — 3×4 panels (plasticity × metric), boxplots by init
  - ``death_pathway_distribution.png`` — **proxy "cause" mix** per mother init (100% stacked)
  - ``death_pathway_counts.csv`` — counts per (plasticity, init, proxy pathway)
  - ``terminal_state_hist_by_mother_type.png`` — overlaid **density histograms** by init
  - ``terminal_injury_by_seed.png`` — one line per init, x = evolve_seed, y = mean terminal injury
  - ``terminal_hunger_by_seed.png`` — same for hunger

Usage::

  python3 plot_child_terminal_risk_profile.py \\
      --root FinalGenomeRollouts_normal_seen \\
      --out-dir FinalGenomeRollouts_normal_seen/figures_child_terminal_profile
"""

from __future__ import annotations

import argparse
import glob
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
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "axes.grid": True,
        "grid.color": "0.90",
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

INIT_ORDER = crb.INIT_ORDER
PLAST_ORDER = crb.PLAST_ORDER
INIT_COLORS = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}
RUN_RE = crb.RUN_RE


def _spearman(x: np.ndarray, y: np.ndarray) -> float:
    x = x[np.isfinite(x) & np.isfinite(y)]
    y = y[np.isfinite(x) & np.isfinite(y)]
    if x.size < 6:
        return float("nan")
    rx = pd.Series(x).rank(method="average").to_numpy(dtype=float)
    ry = pd.Series(y).rank(method="average").to_numpy(dtype=float)
    rx -= rx.mean()
    ry -= ry.mean()
    den = float(np.sqrt((rx * rx).sum() * (ry * ry).sum()))
    if den <= 0:
        return float("nan")
    return float((rx * ry).sum() / den)


def _first_child_death_index(hunger: np.ndarray) -> int | None:
    """Index of first non-finite c0_hunger, or None if child never dies in log."""
    bad = ~np.isfinite(hunger)
    if not bad.any():
        return None
    return int(np.argmax(bad))


def _terminal_window_means(df: pd.DataFrame, *, window: int) -> dict[str, float]:
    hunger = pd.to_numeric(df["c0_hunger"], errors="coerce").to_numpy(dtype=float)
    death_i = _first_child_death_index(hunger)
    if death_i is None:
        lo = max(0, len(hunger) - window)
        hi = len(hunger)
    else:
        last_alive = death_i - 1
        if last_alive < 0:
            nan5 = {k: float("nan") for k in ("t_mean", "w_mean", "i_mean", "f_mean", "rho_fear_injury")}
            nan5["child_death_tick"] = float("nan")
            nan5["n_terminal_ticks"] = 0.0
            return nan5
        lo = max(0, last_alive - window + 1)
        hi = death_i

    sl = slice(lo, hi)
    t = pd.to_numeric(df["c0_hunger"], errors="coerce").to_numpy(dtype=float)[sl]
    w = pd.to_numeric(df["c0_warmth"], errors="coerce").to_numpy(dtype=float)[sl]
    inj = pd.to_numeric(df["c0_injury"], errors="coerce").to_numpy(dtype=float)[sl]
    fear = pd.to_numeric(df["m0_fear_threat"], errors="coerce").to_numpy(dtype=float)[sl]

    def _safe_mean(arr: np.ndarray) -> float:
        a = arr[np.isfinite(arr)]
        return float(np.mean(a)) if a.size else float("nan")

    alive_m = np.isfinite(t) & np.isfinite(w) & np.isfinite(inj)
    out: dict[str, float] = {
        "t_mean": _safe_mean(t[alive_m]) if alive_m.any() else float("nan"),
        "w_mean": _safe_mean(w[alive_m]) if alive_m.any() else float("nan"),
        "i_mean": _safe_mean(inj[alive_m]) if alive_m.any() else float("nan"),
        "f_mean": _safe_mean(fear[alive_m]) if alive_m.any() else float("nan"),
    }

    # alive child ticks for rho (full trajectory, not only terminal window)
    ch_ok = np.isfinite(hunger)
    fi = pd.to_numeric(df["m0_fear_threat"], errors="coerce").to_numpy(dtype=float)
    ci = pd.to_numeric(df["c0_injury"], errors="coerce").to_numpy(dtype=float)
    m = ch_ok & np.isfinite(fi) & np.isfinite(ci)
    out["rho_fear_injury"] = _spearman(fi[m], ci[m])
    out["child_death_tick"] = float(death_i + 1) if death_i is not None else float("nan")
    out["n_terminal_ticks"] = float(int(np.sum(alive_m)))
    return out


def _collect_root(root: str, *, window: int) -> pd.DataFrame:
    root = os.path.abspath(root)
    paths = sorted(glob.glob(os.path.join(root, "*", "run_*.csv")))
    if not paths:
        raise SystemExit(f"No run_*.csv under {root}/*/")

    by_key: dict[tuple[str, str, int], list[dict]] = {}
    skipped = 0
    for p in paths:
        rd = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(rd)
        if not m:
            skipped += 1
            continue
        try:
            df = pd.read_csv(p)
        except Exception:
            skipped += 1
            continue
        need = ("c0_hunger", "c0_warmth", "c0_injury", "m0_fear_threat")
        if any(c not in df.columns for c in need):
            skipped += 1
            continue
        met = _terminal_window_means(df, window=window)
        met["path"] = p
        met["plasticity"] = m.group("plast")
        met["init"] = m.group("init")
        met["evolve_seed"] = int(m.group("seed"))
        key = (met["plasticity"], met["init"], met["evolve_seed"])
        by_key.setdefault(key, []).append(met)

    rows = []
    for (plast, init, seed), lst in by_key.items():
        acc = {k: [] for k in ("t_mean", "w_mean", "i_mean", "f_mean", "rho_fear_injury", "child_death_tick")}
        for r in lst:
            for k in acc:
                acc[k].append(r[k])
        rows.append(
            {
                "plasticity": plast,
                "init": init,
                "evolve_seed": seed,
                "n_runs": len(lst),
                "terminal_hunger_mean": float(np.nanmean(acc["t_mean"])),
                "terminal_warmth_mean": float(np.nanmean(acc["w_mean"])),
                "terminal_injury_mean": float(np.nanmean(acc["i_mean"])),
                "terminal_mother_fear_mean": float(np.nanmean(acc["f_mean"])),
                "spearman_fear_injury_alive": float(np.nanmean(acc["rho_fear_injury"])),
                "median_child_death_tick": float(np.nanmedian(acc["child_death_tick"])),
            }
        )

    out_df = pd.DataFrame(rows)
    out_df["death_pathway_proxy"] = out_df.apply(_death_pathway_proxy, axis=1)
    print(f"[ok] {root}: {len(out_df)} genomes from {len(paths)} runs (skipped dirs/runs={skipped})")
    return out_df


def _death_pathway_proxy(row: pd.Series) -> str:
    """
    Coarse proxy for "which child stress axis dominated right before logged death"
    (not a literal medical cause; child row ends when c0_hunger becomes non-finite).

    Scores in ~[0,1]: hunger/100, injury/100, coldness from warmth (50 = comfortable).
    """
    h = float(pd.to_numeric(row.get("terminal_hunger_mean"), errors="coerce"))
    w = float(pd.to_numeric(row.get("terminal_warmth_mean"), errors="coerce"))
    inj = float(pd.to_numeric(row.get("terminal_injury_mean"), errors="coerce"))
    if not (np.isfinite(h) and np.isfinite(w) and np.isfinite(inj)):
        return "unknown"
    s_h = float(np.clip(h / 100.0, 0.0, 1.0))
    s_i = float(np.clip(inj / 100.0, 0.0, 1.0))
    # warmth 50 ideal → cold stress rises as warmth falls
    s_c = float(np.clip((50.0 - w) / 50.0, 0.0, 1.0))
    scores = {"hunger_stress": s_h, "injury_stress": s_i, "cold_stress": s_c}
    order = ("hunger_stress", "injury_stress", "cold_stress")
    mx = max(scores.values())
    if mx < 0.08:
        return "low_all_axes"
    # argmax; on tie prefer hunger > injury > cold (lower index wins)
    best = max(order, key=lambda k: (scores[k], -order.index(k)))
    return best


def _box_panel(ax, df: pd.DataFrame, *, col: str, ylabel: str) -> None:
    data = []
    for init in INIT_ORDER:
        v = pd.to_numeric(df.loc[df["init"] == init, col], errors="coerce").dropna().to_numpy(dtype=float)
        data.append(v)
    bp = ax.boxplot(
        data,
        labels=[s.replace("_", " ") for s in INIT_ORDER],
        patch_artist=True,
        widths=0.55,
        medianprops=dict(color="#F28E2B", linewidth=1.5),
    )
    for patch, init in zip(bp["boxes"], INIT_ORDER):
        patch.set_facecolor(INIT_COLORS[init])
        patch.set_alpha(0.35)
    ax.set_ylabel(ylabel)


def plot_terminal_grid(df: pd.DataFrame, *, window: int, out_png: str, title: str) -> None:
    metrics = [
        ("terminal_hunger_mean", "c0_hunger"),
        ("terminal_warmth_mean", "c0_warmth"),
        ("terminal_injury_mean", "c0_injury"),
        ("terminal_mother_fear_mean", "m0_fear_threat"),
    ]
    fig, axes = plt.subplots(3, 4, figsize=(14.5, 10.0), sharey=False)
    for i, plast in enumerate(PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        for j, (col, short_name) in enumerate(metrics):
            ax = axes[i, j]
            ylab = f"mean in terminal window\n({short_name})" if j == 0 else ""
            _box_panel(ax, sub, col=col, ylabel=ylab)
            ax.set_title(f"{plast}\n{short_name}", fontsize=9)
    fig.suptitle(f"{title}\nW={window} ticks before first non-finite c0_hunger (child death in log)", fontsize=11, y=1.01)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def plot_by_seed(df: pd.DataFrame, *, col: str, ylabel: str, title: str, out_png: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 3.8), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        for init in INIT_ORDER:
            s = sub[sub["init"] == init].sort_values("evolve_seed")
            if s.empty:
                continue
            ax.plot(
                s["evolve_seed"],
                pd.to_numeric(s[col], errors="coerce"),
                "-o",
                color=INIT_COLORS[init],
                linewidth=1.6,
                markersize=5,
                label=init.replace("_", " "),
            )
        ax.set_xlabel("evolve_seed (genome)")
        ax.set_ylabel(ylabel)
        ax.set_title(plast)
    axes[0].legend(loc="best", fontsize=8, frameon=False)
    fig.suptitle(title, fontsize=11, y=1.05)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def plot_rho_grid(df: pd.DataFrame, *, out_png: str, title: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 3.6), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        _box_panel(ax, sub, col="spearman_fear_injury_alive", ylabel="Spearman ρ (fear, injury)\nchild-alive ticks")
        ax.set_title(plast)
    fig.suptitle(title + "\n(full run, ticks where child hunger finite)", fontsize=10, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


PATHWAY_ORDER = ("hunger_stress", "injury_stress", "cold_stress", "low_all_axes", "unknown")
PATHWAY_COLORS = {
    "hunger_stress": "#d62728",
    "injury_stress": "#9467bd",
    "cold_stress": "#1f77b4",
    "low_all_axes": "#7f7f7f",
    "unknown": "#bcbd22",
}
PATHWAY_LABELS = {
    "hunger_stress": "Hunger (late)",
    "injury_stress": "Injury (late)",
    "cold_stress": "Cold (late)",
    "low_all_axes": "Low all axes",
    "unknown": "Unknown",
}


def plot_death_pathway_distribution(df: pd.DataFrame, *, out_png: str, title: str) -> None:
    """
    100% stacked bars: x = mother init type, segments = proxy pathway fractions
    (within each plasticity panel).
    """
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.2), sharey=True)
    x = np.arange(len(INIT_ORDER), dtype=float)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        bottoms = np.zeros(len(INIT_ORDER))
        for cat in PATHWAY_ORDER:
            heights = []
            for init in INIT_ORDER:
                s = sub[sub["init"] == init]
                if len(s) == 0:
                    heights.append(0.0)
                else:
                    heights.append(float((s["death_pathway_proxy"] == cat).mean()))
            heights = np.asarray(heights, dtype=float)
            ax.bar(
                x,
                heights,
                bottom=bottoms,
                width=0.62,
                label=PATHWAY_LABELS.get(cat, cat),
                color=PATHWAY_COLORS.get(cat, "#cccccc"),
                edgecolor="white",
                linewidth=0.6,
            )
            bottoms += heights
        ax.set_xticks(x)
        ax.set_xticklabels([i.replace("_", " ") for i in INIT_ORDER], rotation=15, ha="right")
        ax.set_ylim(0, 1.0)
        ax.set_ylabel("fraction of genomes")
        ax.set_title(plast)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8, frameon=False, title="Proxy pathway")
    fig.suptitle(
        title + "\nProxy from terminal-window means (hunger / injury / cold); not literal ICD cause",
        fontsize=10,
        y=1.05,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def plot_terminal_state_histograms(df: pd.DataFrame, *, out_png: str, title: str) -> None:
    """Overlaid density histograms of terminal child state by mother init type."""
    metrics = [
        ("terminal_hunger_mean", "Terminal mean c0_hunger"),
        ("terminal_warmth_mean", "Terminal mean c0_warmth"),
        ("terminal_injury_mean", "Terminal mean c0_injury"),
    ]
    fig, axes = plt.subplots(3, 3, figsize=(13.0, 9.5), sharex=False, sharey=False)
    for i, plast in enumerate(PLAST_ORDER):
        subp = df[df["plasticity"] == plast]
        for j, (col, ylab) in enumerate(metrics):
            ax = axes[i, j]
            ax.grid(True, color="0.92", linewidth=0.6)
            for init in INIT_ORDER:
                v = pd.to_numeric(subp.loc[subp["init"] == init, col], errors="coerce").dropna().to_numpy(dtype=float)
                if v.size == 0:
                    continue
                ax.hist(
                    v,
                    bins=14,
                    density=True,
                    alpha=0.38,
                    color=INIT_COLORS[init],
                    label=init.replace("_", " "),
                    histtype="stepfilled",
                    edgecolor="none",
                )
            ax.set_ylabel("density")
            ax.set_title(f"{plast} — {ylab.split()[-1]}", fontsize=9)
            if i == 2:
                ax.set_xlabel(ylab.replace("Terminal mean ", ""))
    axes[0, 2].legend(loc="upper right", fontsize=7, frameon=False)
    fig.suptitle(title + "\nDistributions of terminal child physiology by mother init type", fontsize=11, y=1.01)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def write_pathway_counts(df: pd.DataFrame, out_csv: str) -> None:
    g = (
        df.groupby(["plasticity", "init", "death_pathway_proxy"], dropna=False)
        .size()
        .reset_index(name="n_genomes")
    )
    g.to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="Rollout root with <plast>_<init>_seedN/run_*.csv")
    ap.add_argument("--out-dir", default=None, help="Output dir (default <root>/figures_child_terminal_profile)")
    ap.add_argument("--window", type=int, default=50, help="Number of last child-alive ticks (default 50)")
    ap.add_argument("--title", default="", help="Extra title string")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "figures_child_terminal_profile")
    os.makedirs(out_dir, exist_ok=True)

    df = _collect_root(root, window=args.window)
    csv_path = os.path.join(out_dir, "child_terminal_profile_per_genome.csv")
    df.to_csv(csv_path, index=False)
    print("[ok] wrote", csv_path)

    prefix = (args.title + " — ") if args.title else ""
    plot_terminal_grid(
        df,
        window=args.window,
        out_png=os.path.join(out_dir, "terminal_means_by_init.png"),
        title=prefix + os.path.basename(root),
    )
    plot_by_seed(
        df,
        col="terminal_injury_mean",
        ylabel="mean terminal injury",
        title=prefix + "Terminal injury vs evolve_seed (one value per genome)",
        out_png=os.path.join(out_dir, "terminal_injury_by_seed.png"),
    )
    plot_by_seed(
        df,
        col="terminal_hunger_mean",
        ylabel="mean terminal hunger",
        title=prefix + "Terminal hunger vs evolve_seed",
        out_png=os.path.join(out_dir, "terminal_hunger_by_seed.png"),
    )
    plot_rho_grid(
        df,
        out_png=os.path.join(out_dir, "spearman_fear_injury_by_init.png"),
        title=prefix + "Injury ↔ fear coupling by init type",
    )
    write_pathway_counts(df, os.path.join(out_dir, "death_pathway_counts.csv"))
    plot_death_pathway_distribution(
        df,
        out_png=os.path.join(out_dir, "death_pathway_distribution.png"),
        title=prefix + "Proxy mix of late child stress (by mother init type)",
    )
    plot_terminal_state_histograms(
        df,
        out_png=os.path.join(out_dir, "terminal_state_hist_by_mother_type.png"),
        title=prefix + os.path.basename(root),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
