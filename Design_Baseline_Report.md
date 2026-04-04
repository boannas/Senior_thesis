# Baseline Design and Specification

## 1. Problem Framing

The simulation models a mother agent navigating a gridworld while balancing competing motivations: foraging for food, caring for her child, self-preservation, and protection from threats. The mother's behavior emerges from a weighted motivation system where the highest-scoring motivation determines the action at each tick.

The purpose of the baseline is **not** to produce optimal survival or to ensure all agents succeed. Rather, a baseline defines a **known, reproducible operating point** against which future experimental perturbations (e.g., introducing threats, varying food scarcity, enabling learning) can be measured.

Survival differences across mothers are expected and meaningful. A mother whose weights favor foraging will neglect her child; a mother whose weights favor care will succeed at childcare but may deplete her own energy. These outcomes reflect the inherent structure of the motivation system, not experimental error.

Two baselines are defined:

| Baseline | Purpose | Behavior |
|----------|---------|----------|
| **Baseline-0** (controlled) | Isolate the effect of a single environmental change by holding all behavioral parameters constant | Deterministic; all mothers behave identically |
| **Baseline-1** (heterogeneous) | Capture the natural phenotypic diversity arising from genetic variation | Stochastic; mothers exhibit forager or caregiver phenotypes |

Neither baseline represents a "neutral" condition (see §6).

---

## 2. Calibration Methodology

Baseline-0 required calibration because the motivation system contains a structural asymmetry: forage motivation receives a constant ambient signal from `(1 − fear) ≈ 1.0` when no threats are present, while care motivation has no equivalent boost. Setting all weights to 0.5 does not produce balanced behavior — it produces 88% foraging and 3% child survival.

Calibration proceeded in three phases:

| Phase | Parameter | Range | Replicates | Runs | Finding |
|:-----:|-----------|:-----:|:----------:|:----:|---------|
| 1 | α (care scale) | 0.1 – 1.9 (10 points) | 10 | 100 | Transition between failure and success regimes at α ≈ 0.7 |
| 2 | α (refined) | 0.65 – 0.95 (7 points) | 30 | 210 | Bifurcation confirmed; no stable intermediate exists |
| 3 | γ (low_fear weight) | 0.1 – 1.0 (10 points) | 30 | 300 | γ = 0.60 selected as first stable configuration above the tipping point |

**Outcome:** No weight configuration produces stable intermediate survival rates (40–60%). The system transitions abruptly from failure to success. Baseline-0 was therefore set to the first reliably successful configuration: α = 0.70, γ = 0.60.

Baseline-1 requires no calibration. All motivation weights are drawn from U(0,1), producing the system's natural phenotypic diversity.

---

## 3. Parameter Specification

To establish a clear reference point, parameters are strictly separated into four distinct groups. For each group, we define its state under Baseline-0 (controlled) and Baseline-1 (heterogeneous) as either **fixed** (constant, identical across all runs), **tuned** (calibrated to a specific stable value), or **randomized** (drawn from a probability distribution).

### 3.1 Motivation Weights
Control the high-level decision-making priorities of the mother agent (Forage, Care, Self, Protect).

*   **Baseline-0 State:** **Tuned / Fixed**. Most parameters are strictly fixed at 0.50. The specific exceptions are the tuned parameters $\alpha=0.70$ (Care scale) and $\gamma=0.60$ (Forage low_fear).
*   **Baseline-1 State:** **Randomized**. All motivation sub-weights are independently drawn from $U(0,1)$ per replicate, creating natural phenotypic diversity.

| Category | Sub-weight | Baseline-0 | Baseline-1 |
|----------|-----------|:-----:|:------:|
| **Forage** | child_hunger | 0.50 (Fixed) | U(0,1) (Randomized) |
| | energy_deficit | 0.50 (Fixed) | U(0,1) (Randomized) |
| | low_fear | 0.60 (Tuned, $\gamma$) | U(0,1) (Randomized) |
| **Care** | child_warmth | 0.70 (Tuned, $\alpha$) | U(0,1) (Randomized) |
| | closeness_deficit | 0.70 (Tuned, $\alpha$) | U(0,1) (Randomized) |
| | bonding | 0.70 (Tuned, $\alpha$) | U(0,1) (Randomized) |
| **Self** | fatigue | 0.50 (Fixed) | U(0,1) (Randomized) |
| | fear | 0.50 (Fixed) | U(0,1) (Randomized) |
| | stress | 0.50 (Fixed) | U(0,1) (Randomized) |
| **Protect** | child_injury | 0.50 (Fixed) | U(0,1) (Randomized) |
| | fear | 0.50 (Fixed) | U(0,1) (Randomized) |
| | closeness_deficit | 0.50 (Fixed) | U(0,1) (Randomized) |
| | bonding | 0.50 (Fixed) | U(0,1) (Randomized) |

### 3.2 Psychological-State Weights / Parameters
Control how stimuli (needs, threats, deficits) update the internal psychological variables (bonding, fear, stress, oxytocin, cortisol).

*   **Baseline-0 State:** **Fixed**. Strictly locked to exactly 0.50 for all weights to ensure identical, neutral internal state progression.
*   **Baseline-1 State:** **Randomized**. All psychological-state weights are drawn independently from $U(0,1)$, naturally coupling with the motivation diversity.

| Category | Sub-weight | Baseline-0 | Baseline-1 |
|----------|-----------|:-----:|:------:|
| **Child Need** | hunger | 0.50 (Fixed) | U(0,1) (Randomized) |
| | warmth | 0.50 (Fixed) | U(0,1) (Randomized) |
| | injury | 0.50 (Fixed) | U(0,1) (Randomized) |
| **Oxytocin** | closeness_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | decay | 0.50 (Fixed) | U(0,1) (Randomized) |
| **Bonding** | oxytocin_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | child_need_decay | 0.50 (Fixed) | U(0,1) (Randomized) |
| | child_absent_decay| 0.50 (Fixed) | U(0,1) (Randomized) |
| **Cortisol** | threat_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | child_need_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | energy_deficit_gain| 0.50 (Fixed) | U(0,1) (Randomized) |
| | decay | 0.50 (Fixed) | U(0,1) (Randomized) |
| **Stress** | cortisol_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | fear_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | child_need_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | decay | 0.50 (Fixed) | U(0,1) (Randomized) |
| **Fear** | threat_gain | 0.50 (Fixed) | U(0,1) (Randomized) |
| | decay | 0.50 (Fixed) | U(0,1) (Randomized) |

### 3.3 Physiological Dynamics
Govern energy consumption, fatigue accumulation, and the child's needs.

*   **Baseline-0 State:** **Fixed**.
*   **Baseline-1 State:** **Fixed**.
*(Identical for both baselines to ensure baseline survival pressures remain constant.)*

**Mother Dynamics:**
| Parameter | Value |
|-----------|:-----:|
| Move (energy/fatigue) | −0.30 / +0.50 per move |
| Action (energy/fatigue) | −0.10 / +0.10 per action |
| Idle recovery (energy/fatigue)| +0.05 / −1.00 per tick |
| Rest action (energy/fatigue) | +0.10 / −3.00 per tick |

**Child Dynamics:**
| Parameter | Value |
|-----------|:-----:|
| Hunger increase | +0.50 / tick |
| Warmth (carried / alone) | +0.10 / −0.20 per tick |

### 3.4 Environment Parameters
Define the physical gridworld constraints and initial conditions. 

*   **Baseline-0 State:** **Fixed**.
*   **Baseline-1 State:** **Fixed**.

| Parameter | Value |
|-----------|:-----:|
| Grid size | 20 × 20 |
| Food items | 1 (static respawn) |
| Threats | 0 (neutral condition) |
| Perception range | 8 cells (octile) |
| Simulation length | 3000 ticks (30 days) |
| Mother init energy | 100 |

---

## 4. Design Clarifications

To provide full context on the baseline design decisions and methodology:

### Why Equal Weights (0.5) Were Insufficient
Initially, setting every motivation weight to a perfectly neutral 0.50 seemed like the most logical baseline. However, the motivation equations contain a structural asymmetry: foraging receives a constant, ambient signal from `(1 - fear)` when no threats are present. Care has no such continuous baseline signal. Due to this hidden imbalance, a "neutral" 0.50 configuration caused mothers to forage endlessly, leading to an almost 100% child mortality rate. A purely mathematical midpoint translated into a severe behavioral failure.

### Why Alpha and Gamma Were Introduced
Because perfect 0.50 weights failed to produce a viable caregiver, scaling factors were needed to manually locate a stable reference point:
*   **Alpha (Care Scale):** Introduced to uniformly boost the entire care motivation so the mother would actively attempt to keep her child alive.
*   **Gamma (low_fear Scale):** Introduced to specifically throttle the extreme foraging drive caused by the `(1 - fear)` ambient signal without impacting other forage triggers like child hunger.
Tuning these two specific parameters isolated the structural imbalance, allowing us to find a mathematically working configuration for Baseline-0.

### The "Bifurcation Mechanism"
In complex systems, a "bifurcation" describes a tipping point where a microscopic change in a parameter causes the system to violently split into completely different outcomes. In our model, when $\gamma$ was set near exactly 0.50, the survival variance exploded (e.g., $67\% \pm 43\%$). Sometimes families survived effortlessly, and sometimes they perished entirely on replicate seeds that were nearly identical. The behavior fell off a cliff between absolute stability and chaos. We refer to this tipping point as the "bifurcation mechanism," which is driven by the rigid architecture of the `argmax` action selector snapping between Care and Forage.

### Phenotypes vs. Low-Level Actions
We use phenotype distributions (categorizing agents as Foragers or Caregivers) as a high-level heuristic to interpret the agent populations. However, grouping by motivation trait is only a **first-level** behavioral interpretation — it mainly tells us *what the agent wanted to do*. A comprehensive trait analysis in future experiments should be corroborated by examining the **low-level physical actions** (e.g., exact grid cells moved, ticks spent resting, or precise energy consumed over time). Mapping these physical actions provides the ground truth of *what the agent actually did*, acting as a much stronger metric for deeper analysis.

---

## 5. Results and Interpretation

### 5.1 Baseline-0

| Metric | Value |
|--------|:-----:|
| Child survival | 100% ± 0% |
| Mother survival | 100% |
| Forage / Care split | ~37% / ~63% |
| Variance across seeds | 0% |

**Interpretation:** Baseline-0 is a *success-biased operating point*. It should be understood as a controlled reference — not as a prediction of typical behavior or as a claim that the system "works." When experimental perturbations (e.g., introducing threats, reducing food) are applied, performance will degrade from this ceiling. The magnitude of degradation is the quantity of interest.

> [!WARNING]
> Do not interpret Baseline-0 as "optimal" or "the correct configuration." It is the first stable configuration above a structural tipping point and is explicitly biased toward success.

### 5.2 Baseline-1

| Metric | Value |
|--------|:-----:|
| Child survival (mean) | 35% ± 16% |
| Mother survival | 100% |
| Phenotype: forager | ~65% of mothers (0% child survival) |
| Phenotype: caregiver | ~30% of mothers (100% child survival) |
| Phenotype: mixed | ~5% of mothers |

**Interpretation:** Baseline-1 reveals the system's inherent phenotypic diversity. The bimodal distribution (all-or-nothing child survival) is a structural property of the argmax motivation architecture: small weight differences near the care–forage boundary produce large behavioral divergence. This is not noise — it reflects how genetic variation maps to behavioral outcomes in the model.

Phenotype classification thresholds:
- **Forager:** forage% ≥ 70% AND care% ≤ 30%
- **Caregiver:** care% ≥ 50% OR (forage% ≤ 40% AND care% ≥ 30%)
- **Mixed:** neither criterion met

---

## 6. Usage Guidelines for Future Experiments

### 6.1 Using Baseline-0

Baseline-0 is appropriate when the experimental question is:

> *"How does changing variable X affect outcomes, with all other factors held constant?"*

**Protocol:**
1. Run Baseline-0 to confirm the reference outcome (100% survival)
2. Introduce exactly one experimental modification (e.g., `n_threats = 1`, `perception_range = 5`)
3. Run the modified condition with the same seeds and replicate count
4. Report the **difference** from Baseline-0 as the measured effect

**Guidelines:**
- Use the same seed range for comparison runs (e.g., seeds 43–72)
- Do not change Baseline-0 parameters between experiments
- Report Baseline-0 results alongside experimental results in every comparison

> [!IMPORTANT]
> Baseline-0 performance (100% survival) will not hold under more complex conditions. This is expected. The purpose is to establish a ceiling from which degradation is measured, not to claim that the system always succeeds.

### 6.2 Using Baseline-1

Baseline-1 is appropriate when the experimental question is:

> *"Does intervention Y improve outcomes across diverse behavioral phenotypes?"*

**Protocol:**
1. Run Baseline-1 (60 replicates) to establish the phenotype distribution
2. Apply the experimental intervention under the same weight initialization scheme
3. Compare phenotype-specific outcomes (forager survival, caregiver survival, phenotype ratio)
4. Report whether the intervention shifts the phenotype boundary or improves within-phenotype outcomes

**Guidelines:**
- 60 replicates recommended (minimum 30) for stable phenotype statistics
- Always report phenotype distribution alongside aggregate survival
- The aggregate survival rate (35%) is less informative than the per-phenotype breakdown

### 6.3 Comparative Analysis (Both Baselines)

For a complete experimental evaluation, run both baselines:

| Comparison | Question Answered |
|------------|-------------------|
| Experiment vs. Baseline-0 | How much does performance degrade from the controlled ceiling? |
| Experiment vs. Baseline-1 | Does the intervention improve robustness across diverse phenotypes? |
| Baseline-0 vs. Baseline-1 | What is the cost of genetic diversity under neutral environmental conditions? |

---

## 7. Structural Bifurcation

The calibration process revealed that the motivation system exhibits a **structural tipping point** in its parameter space. This is not a failure of the calibration — it is an intrinsic property of the architecture.

**Mechanism:** The motivation scoring formula normalizes each category by its weight sum:

```
motivation_score = 100 × (w₁·s₁ + w₂·s₂ + w₃·s₃) / (w₁ + w₂ + w₃)
```

Two properties of this formula create the bifurcation:

1. **Normalization invariance:** Uniformly scaling all weights within a category cancels algebraically (numerator and denominator scale equally). Only relative changes between individual sub-weights produce measurable effects.

2. **Signal asymmetry:** The forage `low_fear` sub-weight receives a near-constant signal `(1 − fear) ≈ 1.0` in the absence of threats. Care has no equivalent ambient signal. This gives forage a permanent structural advantage that can only be offset by increasing care weights past a critical threshold.

**Consequence:** The parameter space has no smooth neutral zone. The system transitions abruptly from care dominance (child survives) to forage dominance (child dies) over a narrow weight range, with high variance (±45%) at the boundary itself.

A truly neutral baseline (50% survival, low variance) would require architectural changes to the motivation system — not further weight tuning.

---

## 8. Conclusion

| | Baseline-0 | Baseline-1 |
|---|---|---|
| **Type** | Controlled reference | Heterogeneous reference |
| **Weights** | Fixed ($\alpha$ = 0.70, $\gamma$ = 0.60) | Random U(0, 1) |
| **Behavior** | Deterministic | Stochastic, bimodal |
| **Child survival** | 100% | ~35% |
| **Use case** | Measure effect of single-variable changes | Test robustness across phenotypic diversity |
| **Interpretation** | Success-biased ceiling | Natural variability floor |

Both baselines are required for proper experimental evaluation. Neither represents a neutral operating point. Neutrality is not a tunable parameter in this architecture — it is a structural property that would require changes to the motivation normalization scheme or signal design.

All parameters in §3 are locked for the duration of the experimental campaign. Modifying any parameter listed as "Fixed" defines a new experimental condition and must be explicitly reported.
