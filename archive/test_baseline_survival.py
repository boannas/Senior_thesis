"""
Baseline Survival Debug Test — with Plot Generation
=====================================================
Runs the simulation headless (no pygame) with 0 threats for 1000 ticks.
Generates diagnostic plots saved as PNG images.

Usage:
    python test_baseline_survival.py
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from core.config.config import random_unique_positions
from core.seed import init_seed
init_seed(42)

from core.world import World
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving plots
import matplotlib.pyplot as plt
import numpy as np

# ─── Test Configuration ─────────────────────────────────────────────
GRID_W = 20            # Control variable: grid size
GRID_H = 20            # Control variable: grid size
DAY_STEP = 100
MAX_TICKS = 3000       # 30 simulated days
INITIAL_FOOD_COUNT = 1 # Balanced resource: 1 food item

THREAT_START = []      # NO threats for baseline (neutral condition)

occupied = set()

MOTHER_START, occupied = random_unique_positions(
    n=1, grid_w=GRID_W, grid_h=GRID_H, occupied=occupied
)
CHILD_START, occupied = random_unique_positions(
    n=1, grid_w=GRID_W, grid_h=GRID_H, occupied=occupied
)
FOOD_POSITIONS, occupied = random_unique_positions(
    n=INITIAL_FOOD_COUNT, grid_w=GRID_W, grid_h=GRID_H, occupied=occupied
)

OUTPUT_DIR = Path(__file__).parent / "test_results"
OUTPUT_DIR.mkdir(exist_ok=True)

# ─── Create World ────────────────────────────────────────────────────
world = World(
    GRID_W, GRID_H,
    mother_starts=MOTHER_START,
    child_start=CHILD_START,
    food_positions=FOOD_POSITIONS,
    threat_starts=THREAT_START,
    day_step=DAY_STEP,
)

# ─── Recording arrays ───────────────────────────────────────────────
ticks = []
# Mother
m_energy = []; m_fatigue = []; m_bonding = []
m_fear = []; m_stress = []; m_closeness = []
m_oxytocin = []; m_cortisol = []
m_mot_forage = []; m_mot_care = []; m_mot_self = []; m_mot_protect = []
m_selected = []
m_food_inv = []
m_pos_x = []; m_pos_y = []
# Child
c_hunger = []; c_warmth = []; c_injury = []

mother_alive = True
child_alive = True
mother_died_tick = None
child_died_tick = None
mother_death_cause = None
child_death_cause = None

# ─── Run Simulation ─────────────────────────────────────────────────
print(f"Running {MAX_TICKS} tick baseline survival test...")

for tick in range(MAX_TICKS):
    m = world.mothers[0] if world.mothers else None
    c = world.children[0] if world.children else None

    world.step(0.01)

    m_after = world.mothers[0] if world.mothers else None
    c_after = world.children[0] if world.children else None

    ticks.append(tick)

    # Record mother
    if m_after:
        m_energy.append(m_after.energy)
        m_fatigue.append(m_after.fatigue)
        m_bonding.append(m_after.bonding)
        m_fear.append(m_after.fear_threat)
        m_stress.append(m_after.stress)
        m_closeness.append(m_after.closeness_child)
        m_oxytocin.append(m_after.oxytocin)
        m_cortisol.append(m_after.cortisol)
        m_mot_forage.append(m_after.motivations["Forage"])
        m_mot_care.append(m_after.motivations["Care"])
        m_mot_self.append(m_after.motivations["Self"])
        m_mot_protect.append(m_after.motivations["Protect"])
        m_selected.append(m_after.selected_motivation)
        m_food_inv.append(m_after.food_inventory)
        m_pos_x.append(m_after.x)
        m_pos_y.append(m_after.y)
    else:
        # Fill with last known values or NaN
        for arr in [m_energy, m_fatigue, m_bonding, m_fear, m_stress,
                    m_closeness, m_oxytocin, m_cortisol,
                    m_mot_forage, m_mot_care, m_mot_self, m_mot_protect,
                    m_food_inv, m_pos_x, m_pos_y]:
            arr.append(float('nan'))
        m_selected.append("DEAD")
        if mother_alive and mother_died_tick is None:
            mother_died_tick = tick
            mother_death_cause = f"Energy depleted (last energy={m.energy:.1f})" if m else "Unknown"
            mother_alive = False

    # Record child
    if c_after:
        c_hunger.append(c_after.hunger)
        c_warmth.append(c_after.warmth)
        c_injury.append(c_after.injury)
    else:
        for arr in [c_hunger, c_warmth, c_injury]:
            arr.append(float('nan'))
        if child_alive and child_died_tick is None:
            child_died_tick = tick
            if c:
                if c.hunger >= 100:
                    child_death_cause = f"Starvation (hunger={c.hunger:.1f})"
                elif c.warmth <= 0:
                    child_death_cause = f"Hypothermia (warmth={c.warmth:.1f})"
                elif c.warmth >= 100:
                    child_death_cause = f"Overheating (warmth={c.warmth:.1f})"
                elif c.injury >= 100:
                    child_death_cause = f"Injury (injury={c.injury:.1f})"
                else:
                    child_death_cause = f"Unknown (h={c.hunger:.1f}, w={c.warmth:.1f}, i={c.injury:.1f})"
            else:
                child_death_cause = "Unknown"
            child_alive = False

    if tick % 100 == 0:
        status = "ALIVE" if (m_after and c_after) else "DEAD"
        print(f"  Tick {tick:>5}: {status}")

# ─── Console Summary ─────────────────────────────────────────────────
print(f"\n{'='*60}")
print("RESULTS")
print(f"{'='*60}")

if mother_died_tick is not None:
    print(f"  MOTHER DIED at tick {mother_died_tick}: {mother_death_cause}")
else:
    print(f"  MOTHER SURVIVED all {MAX_TICKS} ticks!")

if child_died_tick is not None:
    print(f"  CHILD DIED at tick {child_died_tick}: {child_death_cause}")
else:
    print(f"  CHILD SURVIVED all {MAX_TICKS} ticks!")

# Motivation distribution
sel_counts = {}
for s in m_selected:
    if s != "DEAD":
        sel_counts[s] = sel_counts.get(s, 0) + 1
total = sum(sel_counts.values())
if total > 0:
    print(f"\n  Motivation distribution ({total} ticks):")
    for mot in ["Forage", "Care", "Self", "Protect"]:
        pct = sel_counts.get(mot, 0) / total * 100
        print(f"    {mot:>8}: {pct:5.1f}% ({sel_counts.get(mot, 0)} ticks)")

# ═══════════════════════════════════════════════════════════════════════
# PLOT 1: Mother Physiological & Psychological States
# ═══════════════════════════════════════════════════════════════════════
fig1, axes1 = plt.subplots(4, 2, figsize=(14, 12))
fig1.suptitle("Mother Agent — Internal States Over Time (No Threats)", fontsize=14, fontweight='bold')

state_data = {
    "Energy": m_energy, "Fatigue": m_fatigue,
    "Bonding": m_bonding, "Fear": m_fear,
    "Stress": m_stress, "Closeness to Child": m_closeness,
    "Oxytocin": m_oxytocin, "Cortisol": m_cortisol,
}

colors = ['#2196F3', '#FF5722', '#E91E63', '#F44336',
          '#FF9800', '#4CAF50', '#9C27B0', '#795548']

for ax, (name, data), color in zip(axes1.flatten(), state_data.items(), colors):
    ax.plot(ticks, data, color=color, linewidth=1.2, alpha=0.9)
    ax.set_title(name, fontweight='bold')
    ax.set_ylabel("Value (0–100)")
    ax.set_ylim(-5, 105)
    ax.set_xlim(0, MAX_TICKS)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='red', linestyle='--', alpha=0.3, linewidth=0.8)
    ax.axhline(y=100, color='red', linestyle='--', alpha=0.3, linewidth=0.8)

    # Mark death ticks
    if child_died_tick:
        ax.axvline(x=child_died_tick, color='red', linestyle=':', alpha=0.5, label='Child died')
    if mother_died_tick:
        ax.axvline(x=mother_died_tick, color='darkred', linestyle=':', alpha=0.5, label='Mother died')

axes1[-1, 0].set_xlabel("Tick")
axes1[-1, 1].set_xlabel("Tick")
fig1.tight_layout(rect=[0, 0, 1, 0.96])
fig1.savefig(str(OUTPUT_DIR / "plot1_mother_states.png"), dpi=150, bbox_inches='tight')
print(f"\n  Saved: {OUTPUT_DIR / 'plot1_mother_states.png'}")

# ═══════════════════════════════════════════════════════════════════════
# PLOT 2: Child States
# ═══════════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(1, 3, figsize=(14, 4))
fig2.suptitle("Child Agent — Internal States Over Time (No Threats)", fontsize=14, fontweight='bold')

child_data = {
    "Hunger": (c_hunger, '#FF5722', 100, "Dies at 100"),
    "Warmth": (c_warmth, '#4CAF50', 50, "Ideal=50, Dies at 0 or 100"),
    "Injury": (c_injury, '#F44336', 0, "Ideal=0, Dies at 100"),
}

for ax, (name, (data, color, ideal, note)) in zip(axes2, child_data.items()):
    ax.plot(ticks, data, color=color, linewidth=1.2, alpha=0.9)
    ax.axhline(y=ideal, color='gray', linestyle='--', alpha=0.5, label=f'Ideal={ideal}')
    ax.set_title(f"{name}\n({note})", fontweight='bold', fontsize=10)
    ax.set_ylabel("Value (0–100)")
    ax.set_xlabel("Tick")
    ax.set_ylim(-5, 105)
    ax.set_xlim(0, MAX_TICKS)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    if child_died_tick:
        ax.axvline(x=child_died_tick, color='red', linestyle=':', alpha=0.5)

fig2.tight_layout(rect=[0, 0, 1, 0.92])
fig2.savefig(str(OUTPUT_DIR / "plot2_child_states.png"), dpi=150, bbox_inches='tight')
print(f"  Saved: {OUTPUT_DIR / 'plot2_child_states.png'}")

# ═══════════════════════════════════════════════════════════════════════
# PLOT 3: Motivation Values + Selection Timeline
# ═══════════════════════════════════════════════════════════════════════
fig3, axes3 = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
fig3.suptitle("Mother — Motivation System Over Time (No Threats)", fontsize=14, fontweight='bold')

mot_colors = {"Forage": "#FFC107", "Care": "#4CAF50", "Self": "#2196F3", "Protect": "#F44336"}

# Top: motivation values
for mot, data in [("Forage", m_mot_forage), ("Care", m_mot_care),
                   ("Self", m_mot_self), ("Protect", m_mot_protect)]:
    axes3[0].plot(ticks, data, color=mot_colors[mot], linewidth=1.0, alpha=0.8, label=mot)
axes3[0].set_ylabel("Motivation Score")
axes3[0].set_ylim(-5, 105)
axes3[0].set_title("Motivation Intensities", fontweight='bold')
axes3[0].legend(loc='upper right', ncol=4)
axes3[0].grid(True, alpha=0.3)

# Bottom: selected motivation as color bands
mot_to_num = {"Forage": 0, "Care": 1, "Self": 2, "Protect": 3, "DEAD": -1}
sel_nums = [mot_to_num.get(s, -1) for s in m_selected]

for mot, num in [("Forage", 0), ("Care", 1), ("Self", 2), ("Protect", 3)]:
    band = [1 if s == num else 0 for s in sel_nums]
    axes3[1].fill_between(ticks, 0, band, alpha=0.6, color=mot_colors[mot], label=mot, step='mid')

axes3[1].set_ylabel("Active Motivation")
axes3[1].set_xlabel("Tick")
axes3[1].set_ylim(0, 1.1)
axes3[1].set_title("Selected Motivation (Timeline)", fontweight='bold')
axes3[1].legend(loc='upper right', ncol=4)
axes3[1].grid(True, alpha=0.3)

fig3.tight_layout(rect=[0, 0, 1, 0.94])
fig3.savefig(str(OUTPUT_DIR / "plot3_motivations.png"), dpi=150, bbox_inches='tight')
print(f"  Saved: {OUTPUT_DIR / 'plot3_motivations.png'}")

# ═══════════════════════════════════════════════════════════════════════
# PLOT 4: Motivation Distribution Pie + Key Stats
# ═══════════════════════════════════════════════════════════════════════
fig4, axes4 = plt.subplots(1, 2, figsize=(12, 5))
fig4.suptitle("Baseline Survival Summary (No Threats)", fontsize=14, fontweight='bold')

# Pie chart
if sel_counts:
    labels = list(sel_counts.keys())
    sizes = [sel_counts[k] for k in labels]
    pie_colors = [mot_colors.get(k, '#999') for k in labels]
    axes4[0].pie(sizes, labels=labels, colors=pie_colors, autopct='%1.1f%%',
                  startangle=90, textprops={'fontsize': 11})
    axes4[0].set_title("Motivation Distribution", fontweight='bold')

# Stats table
col_labels = ["Metric", "Value"]
valid_energy = [v for v in m_energy if not np.isnan(v)]
valid_fatigue = [v for v in m_fatigue if not np.isnan(v)]
valid_hunger = [v for v in c_hunger if not np.isnan(v)]
valid_warmth = [v for v in c_warmth if not np.isnan(v)]

table_data = [
    ["Mother survived", f"{'YES ✓' if mother_died_tick is None else f'NO — died tick {mother_died_tick}'}"],
    ["Child survived", f"{'YES ✓' if child_died_tick is None else f'NO — died tick {child_died_tick}'}"],
    ["Total ticks", str(MAX_TICKS)],
    ["", ""],
    ["M energy (avg)", f"{np.mean(valid_energy):.1f}" if valid_energy else "N/A"],
    ["M energy (min)", f"{np.min(valid_energy):.1f}" if valid_energy else "N/A"],
    ["M fatigue (avg)", f"{np.mean(valid_fatigue):.1f}" if valid_fatigue else "N/A"],
    ["M fatigue (max)", f"{np.max(valid_fatigue):.1f}" if valid_fatigue else "N/A"],
    ["", ""],
    ["C hunger (avg)", f"{np.mean(valid_hunger):.1f}" if valid_hunger else "N/A"],
    ["C hunger (max)", f"{np.max(valid_hunger):.1f}" if valid_hunger else "N/A"],
    ["C warmth (avg)", f"{np.mean(valid_warmth):.1f}" if valid_warmth else "N/A"],
    ["C warmth (min)", f"{np.min(valid_warmth):.1f}" if valid_warmth else "N/A"],
]

if child_died_tick:
    table_data.append(["", ""])
    table_data.append(["Death cause", child_death_cause or "Unknown"])

axes4[1].axis('off')
table = axes4[1].table(
    cellText=table_data,
    colLabels=col_labels,
    loc='center',
    cellLoc='left',
    colWidths=[0.45, 0.55],
)
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.0, 1.4)

# Color survived/died cells
for i, row in enumerate(table_data):
    if 'YES' in str(row[1]):
        table[i+1, 1].set_facecolor('#C8E6C9')
    elif 'NO' in str(row[1]):
        table[i+1, 1].set_facecolor('#FFCDD2')

fig4.tight_layout(rect=[0, 0, 1, 0.92])
fig4.savefig(str(OUTPUT_DIR / "plot4_summary.png"), dpi=150, bbox_inches='tight')
print(f"  Saved: {OUTPUT_DIR / 'plot4_summary.png'}")

# ═══════════════════════════════════════════════════════════════════════
# PLOT 5: Energy vs Food flow (supply chain analysis)
# ═══════════════════════════════════════════════════════════════════════
fig5, axes5 = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
fig5.suptitle("Resource Flow Analysis (No Threats)", fontsize=14, fontweight='bold')

# Mother energy
axes5[0].plot(ticks, m_energy, color='#2196F3', linewidth=1.2, label='Mother Energy')
axes5[0].axhline(y=80, color='gray', linestyle='--', alpha=0.4, label='Ideal=80')
axes5[0].axhline(y=0, color='red', linestyle='--', alpha=0.4, label='Death=0')
axes5[0].set_ylabel("Energy")
axes5[0].set_ylim(-5, 105)
axes5[0].set_title("Mother Energy (loses 1/move, +5/eat)", fontweight='bold')
axes5[0].legend(loc='upper right')
axes5[0].grid(True, alpha=0.3)

# Child hunger
axes5[1].plot(ticks, c_hunger, color='#FF5722', linewidth=1.2, label='Child Hunger')
axes5[1].axhline(y=100, color='red', linestyle='--', alpha=0.4, label='Death=100')
axes5[1].set_ylabel("Hunger")
axes5[1].set_ylim(-5, 105)
axes5[1].set_title("Child Hunger (+1/tick, -20/feed)", fontweight='bold')
axes5[1].legend(loc='upper right')
axes5[1].grid(True, alpha=0.3)

# Food inventory
axes5[2].plot(ticks, m_food_inv, color='#4CAF50', linewidth=1.2, label='Food Inventory')
axes5[2].set_ylabel("Count")
axes5[2].set_xlabel("Tick")
axes5[2].set_title("Mother Food Inventory", fontweight='bold')
axes5[2].legend(loc='upper right')
axes5[2].grid(True, alpha=0.3)

fig5.tight_layout(rect=[0, 0, 1, 0.95])
fig5.savefig(str(OUTPUT_DIR / "plot5_resource_flow.png"), dpi=150, bbox_inches='tight')
print(f"  Saved: {OUTPUT_DIR / 'plot5_resource_flow.png'}")

plt.close('all')
print(f"\nAll plots saved to: {OUTPUT_DIR}")
print("DONE.")
