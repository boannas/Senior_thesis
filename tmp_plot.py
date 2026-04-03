import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Set paths
BASE_DIR = r"c:\Users\User\Desktop\FIBO_Study\3Y_2\FRA361_OPENTOPICS\Thesis_Farao\Senior_thesis"
b0_path = os.path.join(BASE_DIR, "test_results/baseline_0/baseline_0_replicates.csv")
b1_path = os.path.join(BASE_DIR, "test_results/baseline_1/baseline_1_replicates.csv")
output_dir = os.path.join(BASE_DIR, "plots/baseline_clean")

os.makedirs(output_dir, exist_ok=True)

# Read data
df0 = pd.read_csv(b0_path)
df1 = pd.read_csv(b1_path)

# Extract survival percentage (0 or 100)
surv0 = df0['child_survival'].values * 100
surv1 = df1['child_survival'].values * 100

# Calculate stats
mean0, std0 = np.mean(surv0), np.std(surv0)
mean1, std1 = np.mean(surv1), np.std(surv1)

# Prepare DataFrame for seaborn
data = []
for v in surv0:
    data.append({'Baseline': 'Baseline-0\n(α=0.70, γ=0.60)', 'Survival': v})
for v in surv1:
    data.append({'Baseline': 'Baseline-1\n(Random Initialization)', 'Survival': v})
df_plot = pd.DataFrame(data)

# Visual setup
sns.set_theme(style="whitegrid", rc={"axes.grid.axis": "y", "grid.alpha": 0.3})
colors = sns.color_palette("muted")[0:2]

# ---------------------------------------------------------
# Plot 1: Comparison
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 6), dpi=300)

# Barplot for mean (with standard deviation error bands via ci='sd')
# ci='sd' in older seaborn, errorbar='sd' in newer. Using errorbar to be safe or ci depending on version.
# Let's just use barplot but turn off error bars, then draw them manually for custom styling
sns.barplot(data=df_plot, x='Baseline', y='Survival', palette=colors, alpha=0.6, errorbar=None, ax=ax, width=0.5)

# Add custom thin error bars
x_pos = [0, 1]
means = [mean0, mean1]
stds = [std0, std1]
ax.errorbar(x_pos, means, yerr=stds, fmt='none', ecolor='black', elinewidth=1.5, capsize=5, capthick=1.5, alpha=0.7)

# Overlay individual data points (jitter)
sns.stripplot(data=df_plot, x='Baseline', y='Survival', color='black', alpha=0.3, jitter=0.15, size=4, ax=ax)

# Horizontal reference line
ax.axhline(50, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax.text(1.4, 52, "Neutral Target (50%)", color='gray', va='bottom', ha='right', fontsize=10, style='italic')

# Labels and stats annotations
ax.set_title("Baseline Comparison: Controlled vs Stochastic Behavior", fontsize=14, fontweight='bold', pad=20)
ax.set_ylabel("Child Survival (%)", fontsize=12)
ax.set_xlabel("")
ax.set_ylim(-5, 120)

ax.text(0, 105, f"{mean0:.1f}% ± {std0:.1f}%", ha='center', va='bottom', fontsize=11, fontweight='bold')
ax.text(1, 105, f"{mean1:.1f}% ± {std1:.1f}%", ha='center', va='bottom', fontsize=11, fontweight='bold')

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "baseline_comparison_academic.png"))
plt.close()

# ---------------------------------------------------------
# Plot 2: Baseline-1 Distribution
# ---------------------------------------------------------
fig2, ax2 = plt.subplots(figsize=(7, 5), dpi=300)

# KDE + Histogram for phenotype breakdown
bins = [0, 50, 100]  # rough binning for 0 and 100
sns.histplot(surv1, bins=10, kde=True, color=colors[1], alpha=0.5, ax=ax2)

# Annotate Bimodal
ax2.set_title("Baseline-1 Phenotype Distribution", fontsize=14, fontweight='bold', pad=15)
ax2.set_xlabel("Child Survival (%)", fontsize=12)
ax2.set_ylabel("Frequency", fontsize=12)
ax2.text(mean1, ax2.get_ylim()[1]*0.8, "Bimodal Behavior\n(~70% Forager, ~30% Caregiver)",
         ha='center', va='center', bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "baseline1_distribution_clean.png"))
plt.close()
