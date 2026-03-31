"""
Plot experiment results. Reads scenario from func/experiment_config.yaml (scenario_name)
and loads func/experiment_plasticity_{scenario_name}_summary.csv (and _timeseries.csv).
Override with: python plot_experiment_plasticity.py --scenario high_threat
"""
import sys
import argparse
import yaml
from pathlib import Path
import numpy as np

try:
    import pandas as pd
    import matplotlib.pyplot as plt
except ImportError as e:
    print("Need pandas and matplotlib: pip install pandas matplotlib")
    raise

# Project root
ROOT = Path(__file__).resolve().parent
FUNC_DIR = ROOT / "func"
EXPERIMENT_CONFIG_PATH = FUNC_DIR / "experiment_config.yaml"
OUTPUT_DIR = FUNC_DIR


def get_scenario_name():
    """Scenario name from config file or --scenario. Default 'baseline'."""
    parser = argparse.ArgumentParser(description="Plot plasticity experiment results")
    parser.add_argument("--scenario", "-s", type=str, default=None, help="Scenario name (overrides config)")
    args, _ = parser.parse_known_args()
    if args.scenario:
        return args.scenario
    if not EXPERIMENT_CONFIG_PATH.is_file():
        return "baseline"
    with open(EXPERIMENT_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("scenario_name", "baseline")

# Fallback when CSV has no plasticity_rule/eta
CONDITION_ORDER_LEGACY = ["fixed", "outcome", "hebbian"]
CONDITION_LABELS_LEGACY = {"fixed": "Fixed", "outcome": "Outcome-gated", "hebbian": "Hebbian"}
COLORS_BY_RULE = {"fixed": "#6c757d", "outcome": "#0d6efd", "hebbian": "#198754"}
# Distinct palette so each condition (e.g. outcome_eta0.01, hebbian_eta0.02) is visible
PALETTE = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]


def get_summary_and_timeseries_paths(scenario_name):
    return (
        FUNC_DIR / f"experiment_plasticity_{scenario_name}_summary.csv",
        FUNC_DIR / f"experiment_plasticity_{scenario_name}_timeseries.csv",
    )


def load_summary(scenario_name=None):
    if scenario_name is None:
        scenario_name = get_scenario_name()
    summary_path, _ = get_summary_and_timeseries_paths(scenario_name)
    if not summary_path.is_file():
        raise FileNotFoundError("Run run_experiment_plasticity.py first to generate " + str(summary_path))
    return pd.read_csv(summary_path)


def load_timeseries(scenario_name=None):
    if scenario_name is None:
        scenario_name = get_scenario_name()
    _, timeseries_path = get_summary_and_timeseries_paths(scenario_name)
    if not timeseries_path.is_file():
        return None
    return pd.read_csv(timeseries_path)


def get_condition_order_and_colors(df):
    """Derive condition order and colors from CSV. Handles optional plasticity_rule and eta."""
    if "condition" not in df.columns:
        return CONDITION_ORDER_LEGACY, [COLORS_BY_RULE.get(c, "gray") for c in CONDITION_ORDER_LEGACY]
    order = df["condition"].unique().tolist()
    # Sort: fixed first, then outcome (by eta), then hebbian (by eta)
    def key(c):
        s = str(c).lower()
        if s == "fixed":
            return (0, 0)
        if s.startswith("outcome"):
            eta = _eta_from_condition(df, c)
            return (1, eta if eta is not None else 0)
        if s.startswith("hebbian"):
            eta = _eta_from_condition(df, c)
            return (2, eta if eta is not None else 0)
        return (3, 0)
    order = sorted(order, key=key)
    # One color per condition from PALETTE so all lines/bars are distinguishable (not all gray)
    n = len(order)
    colors = [PALETTE[i % len(PALETTE)] for i in range(n)]
    return order, colors


def _eta_from_condition(df, cond):
    if "eta" not in df.columns:
        return None
    sub = df[df["condition"] == cond]
    if sub.empty:
        return None
    v = sub["eta"].iloc[0]
    if v == "" or (isinstance(v, float) and np.isnan(v)):
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None

def plot_survival_bars(df, max_tick=1000):
    """Bar chart: mother and child survival tick by condition (mean ± SD)."""
    condition_order, colors = get_condition_order_and_colors(df)
    n = len(condition_order)

    mother_max = max_tick
    child_max = max_tick // 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(max(8, n * 0.8), 4))
    x = np.arange(n)
    width = max(0.35, 0.8 / n)

    configs = [
        (ax1, "mother_survival_tick", "Mother survival tick", mother_max),
        (ax2, "child_survival_tick", "Child survival tick", child_max),
    ]

    for ax, metric, ylabel, max_val in configs:
        means = [df[df["condition"] == c][metric].mean() for c in condition_order]
        sds = [df[df["condition"] == c][metric].std(ddof=1) for c in condition_order]

        ax.bar(
            x, means, width,
            yerr=sds, capsize=3,
            color=colors, edgecolor="black", linewidth=0.5
        )

        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(condition_order, rotation=45, ha="right")

        ax.set_ylim(0, max_val * 1.05)
        ax.axhline(max_val, color="gray", linestyle="--", alpha=0.7)

        ax.grid(axis="y", alpha=0.3)
        ax.set_title(ylabel + " (higher = better)")

    fig.suptitle("Plasticity experiment: 1 mother, 1 child, 1 threat, 1000 ticks (mean ± SD)")
    plt.tight_layout()
    return fig


def plot_mean_deficit_bar(df):
    """Bar chart: mean deficit over run by condition (lower = better)."""
    condition_order, colors = get_condition_order_and_colors(df)
    n = len(condition_order)
    fig, ax = plt.subplots(figsize=(max(5, n * 0.6), 4))
    x = np.arange(n)
    means = [df[df["condition"] == c]["mean_deficit"].mean() for c in condition_order]
    sds = [df[df["condition"] == c]["mean_deficit"].std(ddof=1) for c in condition_order]
    ax.bar(x, means, max(0.35, 0.8 / n), yerr=sds, capsize=3, color=colors, edgecolor="black", linewidth=0.5)
    ax.set_ylabel("Mean overall deficit")
    ax.set_xticks(x)
    ax.set_xticklabels(condition_order, rotation=45, ha="right")
    ax.set_title("Mean deficit over run (lower = better)")
    ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    return fig


def plot_survival_curves(ts, condition_order=None, colors=None):
    """Line plot: proportion of runs with mother/child alive at each tick, by condition."""
    if ts is None or ts.empty:
        return None
    if condition_order is None:
        condition_order = ts["condition"].unique().tolist()
    if colors is None:
        colors = [PALETTE[i % len(PALETTE)] for i in range(len(condition_order))]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    for i, cond in enumerate(condition_order):
        sub = ts[ts["condition"] == cond]
        if sub.empty:
            continue
        c = colors[i % len(colors)]
        mom = sub.groupby("tick")["mother_alive"].mean()
        child = sub.groupby("tick")["child_alive"].mean()
        ax1.plot(mom.index, mom.values, color=c, label=cond)
        ax2.plot(child.index, child.values, color=c, label=cond)
    ax1.set_xlabel("Tick")
    ax1.set_ylabel("Proportion mother alive")
    ax1.set_title("Mother survival over time")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax1.set_ylim(-0.05, 1.05)
    ax2.set_xlabel("Tick")
    ax2.set_ylabel("Proportion child alive")
    ax2.set_title("Child survival over time")
    ax2.legend()
    ax2.grid(alpha=0.3)
    ax2.set_ylim(-0.05, 1.05)
    fig.suptitle("Survival curves (1000 steps)")
    plt.tight_layout()
    return fig


def plot_deficit_over_time(ts, condition_order=None, colors=None):
    """Line plot: mean deficit over tick by condition."""
    if ts is None or ts.empty:
        return None
    if condition_order is None:
        condition_order = ts["condition"].unique().tolist()
    if colors is None:
        colors = [PALETTE[i % len(PALETTE)] for i in range(len(condition_order))]
    fig, ax = plt.subplots(figsize=(8, 5))
    for i, cond in enumerate(condition_order):
        sub = ts[ts["condition"] == cond]
        if sub.empty:
            continue
        c = colors[i % len(colors)]
        by_tick = sub.groupby("tick")["deficit"].mean()
        ax.plot(by_tick.index, by_tick.values, color=c, label=cond, linewidth=1.5)
    ax.set_xlabel("Tick")
    ax.set_ylabel("Mean overall deficit")
    ax.set_title("Mean deficit over time (lower = better)")
    n_cond = len(condition_order)
    ax.legend(loc="best", fontsize=8, ncol=2 if n_cond > 4 else 1)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    return fig


MOTIVATION_COLORS = {"Forage": "#2ca02c", "Care": "#1f77b4", "Self": "#ff7f0e", "Protect": "#d62728"}


def plot_motivation_distribution(df, condition_order=None, colors=None):
    """Stacked bar: proportion of ticks each motivation was selected, by condition (mean across runs)."""
    for col in ("p_Forage", "p_Care", "p_Self", "p_Protect"):
        if col not in df.columns:
            return None
    if condition_order is None:
        condition_order, _ = get_condition_order_and_colors(df)
    n = len(condition_order)
    fig, ax = plt.subplots(figsize=(max(6, n * 0.8), 5))
    x = np.arange(n)
    width = 0.6
    bottom = np.zeros(n)
    for mot, c in MOTIVATION_COLORS.items():
        col = f"p_{mot}"
        means = [df[df["condition"] == cond][col].mean() for cond in condition_order]
        ax.bar(x, means, width, bottom=bottom, color=c, label=mot, edgecolor="white", linewidth=0.5)
        bottom += np.array(means)
    ax.set_ylabel("Proportion of ticks")
    ax.set_xticks(x)
    ax.set_xticklabels(condition_order, rotation=45, ha="right")
    ax.set_title("Selected motivation distribution (over run)")
    ax.legend(loc="upper right", ncol=2)
    ax.set_ylim(0, 1.02)
    ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    return fig


def plot_motivation_over_time(ts, condition_order=None):
    """Line plot: proportion of runs that selected each motivation at each tick (one line per motivation, first condition)."""
    if ts is None or ts.empty or "selected_motivation" not in ts.columns:
        return None
    if condition_order is None:
        condition_order = ts["condition"].unique().tolist()
    ref_cond = condition_order[0]
    sub = ts[ts["condition"] == ref_cond]
    if sub.empty:
        return None
    fig, ax = plt.subplots(figsize=(8, 5))
    for mot in ("Forage", "Care", "Self", "Protect"):
        c = MOTIVATION_COLORS.get(mot, "gray")
        sel = (sub["selected_motivation"] == mot).astype(float)
        by_tick = sub.groupby("tick")["selected_motivation"].apply(lambda x: (x == mot).mean()).reset_index()
        by_tick.columns = ["tick", "prop"]
        ax.plot(by_tick["tick"], by_tick["prop"], color=c, linewidth=1.5, label=mot)
    ax.set_xlabel("Tick")
    ax.set_ylabel("Proportion of runs selecting motivation")
    ax.set_title(f"Selected motivation over time (condition: {ref_cond})")
    ax.legend()
    ax.set_ylim(-0.05, 1.05)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    return fig


def plot_eta_comparison(df, max_tick=1000):
    """
    Line plot: effect of learning rate (eta) on survival.
    Only plasticity runs with numeric eta. One line for outcome, one for hebbian.
    """
    if "eta" not in df.columns:
        return None
    # Rows with numeric eta (exclude fixed / empty)
    df_eta = df.copy()
    df_eta["eta_num"] = pd.to_numeric(df_eta["eta"], errors="coerce")
    df_eta = df_eta.dropna(subset=["eta_num"])
    if df_eta.empty:
        return None
    plasticity_rule = df_eta["plasticity_rule"] if "plasticity_rule" in df_eta.columns else df_eta["condition"]
    df_eta = df_eta[plasticity_rule.isin(["outcome", "hebbian"])]
    if df_eta.empty:
        return None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    for rule in ["outcome", "hebbian"]:
        sub = df_eta[df_eta["plasticity_rule"] == rule]
        if sub.empty:
            continue
        by_eta = sub.groupby("eta_num").agg(
            mother_mean=("mother_survival_tick", "mean"),
            mother_sd=("mother_survival_tick", "std"),
            child_mean=("child_survival_tick", "mean"),
            child_sd=("child_survival_tick", "std"),
        ).reset_index()
        if by_eta.shape[0] == 0:
            continue
        by_eta = by_eta.sort_values("eta_num")
        x = by_eta["eta_num"]
        c = COLORS_BY_RULE.get(rule, "gray")
        label = "Outcome-gated" if rule == "outcome" else "Hebbian"
        ax1.plot(x, by_eta["mother_mean"], "o-", color=c, label=label)
        ax1.fill_between(x, by_eta["mother_mean"] - by_eta["mother_sd"], by_eta["mother_mean"] + by_eta["mother_sd"], color=c, alpha=0.2)
        ax2.plot(x, by_eta["child_mean"], "o-", color=c, label=label)
        ax2.fill_between(x, by_eta["child_mean"] - by_eta["child_sd"], by_eta["child_mean"] + by_eta["child_sd"], color=c, alpha=0.2)
    ax1.set_xlabel("Learning rate (eta)")
    ax1.set_ylabel("Mother survival tick")
    ax1.set_title("Effect of eta (higher = better)")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax1.set_ylim(0, max_tick * 1.05)
    ax2.set_xlabel("Learning rate (eta)")
    ax2.set_ylabel("Child survival tick")
    ax2.set_title("Effect of eta (higher = better)")
    ax2.legend()
    ax2.grid(alpha=0.3)
    ax2.set_ylim(0, max_tick * 1.05)
    fig.suptitle("Learning rate (eta) comparison")
    plt.tight_layout()
    return fig


def main():
    scenario_name = get_scenario_name()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    df = load_summary(scenario_name)
    condition_order, colors = get_condition_order_and_colors(df)
    max_tick = int(df["mother_survival_tick"].max()) if "mother_survival_tick" in df.columns else 1000
    prefix = f"experiment_plasticity_{scenario_name}"

    print("Scenario:", scenario_name)
    print("Summary:", get_summary_and_timeseries_paths(scenario_name)[0])
    print()

    # 1) Survival ticks bar chart
    fig1 = plot_survival_bars(df, max_tick=max_tick)
    out1 = OUTPUT_DIR / f"{prefix}_survival_bars.png"
    fig1.savefig(out1, dpi=150, bbox_inches="tight")
    plt.close(fig1)
    print("Saved", out1)

    # 2) Mean deficit bar chart
    fig2 = plot_mean_deficit_bar(df)
    out2 = OUTPUT_DIR / f"{prefix}_mean_deficit_bars.png"
    fig2.savefig(out2, dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print("Saved", out2)

    # 3) Eta comparison (when CSV has numeric eta)
    fig_eta = plot_eta_comparison(df, max_tick=max_tick)
    if fig_eta is not None:
        out_eta = OUTPUT_DIR / f"{prefix}_eta_comparison.png"
        fig_eta.savefig(out_eta, dpi=150, bbox_inches="tight")
        plt.close(fig_eta)
        print("Saved", out_eta)

    # 4) Selected motivation distribution (stacked bar by condition)
    fig_mot = plot_motivation_distribution(df, condition_order=condition_order, colors=colors)
    if fig_mot is not None:
        out_mot = OUTPUT_DIR / f"{prefix}_motivation_distribution.png"
        fig_mot.savefig(out_mot, dpi=150, bbox_inches="tight")
        plt.close(fig_mot)
        print("Saved", out_mot)

    # 5) Timeseries: survival curves, deficit over time, motivation over time
    ts = load_timeseries(scenario_name)
    if ts is not None and not ts.empty:
        fig3 = plot_survival_curves(ts, condition_order=condition_order, colors=colors)
        if fig3 is not None:
            out3 = OUTPUT_DIR / f"{prefix}_survival_curves.png"
            fig3.savefig(out3, dpi=150, bbox_inches="tight")
            plt.close(fig3)
            print("Saved", out3)
        fig4 = plot_deficit_over_time(ts, condition_order=condition_order, colors=colors)
        if fig4 is not None:
            out4 = OUTPUT_DIR / f"{prefix}_deficit_over_time.png"
            fig4.savefig(out4, dpi=150, bbox_inches="tight")
            plt.close(fig4)
            print("Saved", out4)
        fig5 = plot_motivation_over_time(ts, condition_order=condition_order)
        if fig5 is not None:
            out5 = OUTPUT_DIR / f"{prefix}_motivation_over_time.png"
            fig5.savefig(out5, dpi=150, bbox_inches="tight")
            plt.close(fig5)
            print("Saved", out5)
    else:
        print("No timeseries CSV found; skipping survival curves, deficit-over-time, and motivation-over-time plots.")

    print("Done.")


if __name__ == "__main__":
    main()
