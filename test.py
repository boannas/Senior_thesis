import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# =========================================================
# 1) GENERATION-LEVEL SYNTHETIC DATA
# =========================================================
gens = 100
gen = np.arange(gens)

# ----- Motivation distribution -----
# First 50% random, last 50% gradually converge to equal distribution
dist = []

for g in gen:
    p = g / (gens - 1)

    if p < 0.5:
        sample = np.random.dirichlet([0.2] * 4)
    else:
        t = (p - 0.5) / 0.5
        random_part = np.random.dirichlet([0.2] * 4)
        equal = np.ones(4) / 4
        sample = (1 - t) * random_part + t * equal

    dist.append(sample)

dist = np.array(dist)
m_forage, m_care, m_self, m_protect = dist.T

# ----- Fitness based on closeness to balanced motivation -----
target = np.ones(4) / 4
fitness = 1 - np.sum((dist - target) ** 2, axis=1)
fitness += np.random.normal(0, 0.01, size=gens)
fitness = np.clip(fitness, 0, 1)

# ----- Survival time improves over generations -----
max_time = 200
child_survival = 70 + 110 * (1 - np.exp(-gen / 35)) + np.random.normal(0, 6, size=gens)
child_survival = np.clip(child_survival, 0, max_time)

mother_survival = 90 + 90 * (1 - np.exp(-gen / 40)) + np.random.normal(0, 5, size=gens)
mother_survival = np.clip(mother_survival, 0, max_time)

# ----- Injury decreases over generations -----
child_injury = 0.65 * np.exp(-gen / 30) + 0.12 + np.random.normal(0, 0.02, size=gens)
child_injury = np.clip(child_injury, 0, 1)

# =========================================================
# 2) EXAMPLE EARLY VS LATE TRAJECTORIES
# =========================================================
T = 150
t = np.arange(T)

def smooth_noise(scale: float, size: int, kernel_size: int = 7) -> np.ndarray:
    raw = np.random.normal(0, scale, size=size)
    kernel = np.ones(kernel_size) / kernel_size
    return np.convolve(raw, kernel, mode="same")

# Early generation: less stable
mother_energy_early = 80 - 0.18 * t + 6 * np.sin(t / 8) + smooth_noise(3.5, T)
mother_energy_early = np.clip(mother_energy_early, 0, 100)

mother_fatigue_early = 35 + 0.22 * t + 7 * np.sin(t / 10) + smooth_noise(3.0, T)
mother_fatigue_early = np.clip(mother_fatigue_early, 0, 100)

child_hunger_early = 20 + 0.28 * t + 5 * np.sin(t / 9) + smooth_noise(2.8, T)
child_hunger_early = np.clip(child_hunger_early, 0, 100)

child_warmth_early = 50 - 0.16 * t + 5 * np.sin(t / 11) + smooth_noise(2.5, T)
child_warmth_early = np.clip(child_warmth_early, 0, 100)

child_injury_early = 10 + 0.10 * t + 4 * np.maximum(0, np.sin(t / 7)) + smooth_noise(1.8, T)
child_injury_early = np.clip(child_injury_early, 0, 100)

# Late generation: more stable
mother_energy_late = 78 - 0.05 * t + 2 * np.sin(t / 14) + smooth_noise(1.2, T)
mother_energy_late = np.clip(mother_energy_late, 0, 100)

mother_fatigue_late = 30 + 0.06 * t + 2 * np.sin(t / 16) + smooth_noise(1.0, T)
mother_fatigue_late = np.clip(mother_fatigue_late, 0, 100)

child_hunger_late = 18 + 0.10 * t + 1.8 * np.sin(t / 15) + smooth_noise(1.0, T)
child_hunger_late = np.clip(child_hunger_late, 0, 100)

child_warmth_late = 48 - 0.04 * t + 1.5 * np.sin(t / 17) + smooth_noise(0.9, T)
child_warmth_late = np.clip(child_warmth_late, 0, 100)

child_injury_late = 6 + 0.02 * t + 1.2 * np.maximum(0, np.sin(t / 12)) + smooth_noise(0.7, T)
child_injury_late = np.clip(child_injury_late, 0, 100)

# =========================================================
# 3) PLOT A: EVOLUTIONARY OUTCOMES
# =========================================================
fig1, axs = plt.subplots(2, 2, figsize=(12, 8))

# Fitness
axs[0, 0].plot(gen, fitness)
axs[0, 0].set_title("Fitness Over Generations")
axs[0, 0].set_xlabel("Generation")
axs[0, 0].set_ylabel("Fitness")
axs[0, 0].set_ylim(0, 1.05)

# Survival time
axs[0, 1].plot(gen, child_survival, label="Child")
axs[0, 1].plot(gen, mother_survival, label="Mother")
axs[0, 1].set_title("Survival Time Over Generations")
axs[0, 1].set_xlabel("Generation")
axs[0, 1].set_ylabel("Survival Time")
axs[0, 1].set_ylim(0, max_time + 10)
axs[0, 1].legend()

# Injury
axs[1, 0].plot(gen, child_injury)
axs[1, 0].set_title("Average Child Injury Over Generations")
axs[1, 0].set_xlabel("Generation")
axs[1, 0].set_ylabel("Normalized Injury")
axs[1, 0].set_ylim(0, 1.0)

# Motivation distribution
axs[1, 1].stackplot(
    gen,
    m_forage, m_care, m_self, m_protect,
    labels=["Forage", "Care", "Self", "Protect"]
)
axs[1, 1].set_title("Motivation Distribution Over Generations")
axs[1, 1].set_xlabel("Generation")
axs[1, 1].set_ylabel("Proportion")
axs[1, 1].set_ylim(0, 1.0)
axs[1, 1].legend(loc="upper right")

plt.tight_layout()
plt.show()

# =========================================================
# 4) PLOT B: EARLY VS LATE STATE TRAJECTORIES
# =========================================================
fig2, axs = plt.subplots(2, 3, figsize=(14, 7), sharex=True)

# Mother energy
axs[0, 0].plot(t, mother_energy_early, label="Early")
axs[0, 0].plot(t, mother_energy_late, label="Late")
axs[0, 0].set_title("Mother Energy")
axs[0, 0].set_ylabel("State Value")
axs[0, 0].legend()

# Mother fatigue
axs[0, 1].plot(t, mother_fatigue_early, label="Early")
axs[0, 1].plot(t, mother_fatigue_late, label="Late")
axs[0, 1].set_title("Mother Fatigue")
axs[0, 1].legend()

# Child hunger
axs[0, 2].plot(t, child_hunger_early, label="Early")
axs[0, 2].plot(t, child_hunger_late, label="Late")
axs[0, 2].set_title("Child Hunger")
axs[0, 2].legend()

# Child warmth
axs[1, 0].plot(t, child_warmth_early, label="Early")
axs[1, 0].plot(t, child_warmth_late, label="Late")
axs[1, 0].set_title("Child Warmth")
axs[1, 0].set_xlabel("Time")
axs[1, 0].set_ylabel("State Value")
axs[1, 0].legend()

# Child injury
axs[1, 1].plot(t, child_injury_early, label="Early")
axs[1, 1].plot(t, child_injury_late, label="Late")
axs[1, 1].set_title("Child Injury")
axs[1, 1].set_xlabel("Time")
axs[1, 1].legend()

# Empty panel for cleaner layout or notes
axs[1, 2].axis("off")
axs[1, 2].text(
    0.05, 0.6,
    "Interpretation:\n"
    "- Early generations: noisier, less stable regulation\n"
    "- Late generations: smoother states,\n"
    "  lower injury, better survival",
    fontsize=10
)

plt.tight_layout()
plt.show()