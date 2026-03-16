# Maternal Intuition: Mother Agent Pipeline

This document summarizes how the **mother agent** works: world update, internal states, motivation, genes (weights), and plasticity.

---

## 1. Update rule: how the world updates (one tick)

Each **tick**, the world runs the following order:

```
1. Children update     → child.update(world)  [child states: hunger, warmth, injury]
2. Mothers psych/neuro → mother.update_psych_neuro(world)  [OT, CORT, bonding, fear, stress, closeness]
3. Policy propose      → mother_policy_propose(world)  [motivation_compute → select_motivation → choose_goal → proposals]
4. Move & apply        → resolve_and_apply_moves(); apply_mother_intents()  [positions, feed/care/eat/etc.]
5. Plasticity (optional) → update_plasticity() or update_plasticity_hebbian()  [modify u_plastic, w_plastic]
6. Threat intents      → apply_threat_intents()
7. Cleanup             → cleanup_dead_agents(); cleanup_pick_entities()
8. Spawn food          → spawn_random_food() (if interval)
9. Record history      → record_child_states(); record_mother_states()
10. Advance tick       → t += dt; tick += 1
```

So in one tick: **internal dynamics first** (psych/neuro), then **decide** (motivation + goal), then **act** (move + intents), then **learn** (plasticity), then **environment** (threats, food, record).

---

## 2. Internal states

### 2.1 Physiological (mother)

| State    | Range  | Update (each tick after action) |
|----------|--------|----------------------------------|
| **energy** | [0, 100] | If **moved**: + MOVED_cost["Energy"] = -1.0. If **acted** (care/eat/etc.): + ACTED_cost["Energy"] = -0.1. Else (rest): + RECOVERY_rate["Energy"] = -0.1. Then clamp to [0, 100]. |
| **fatigue** | [0, 100] | If **moved**: + 2.0. If **acted**: + 0.1. Else: -0.1. Clamp [0, 100]. |

So moving is costly (energy down, fatigue up); resting recovers fatigue and slowly drains energy (hunger).

### 2.2 Psychological & hormonal (mother)

Updated in **update_psych_neuro(world)** using weights **w** (see §4). All states clamped to [0, 100].

| State | Meaning | Update (concept) |
|-------|--------|------------------|
| **closeness_child** | Proximity to child (100 = same cell) | 100 × (1 − normalized_distance). |
| **OT** (oxytocin) | Increases when close to child (closeness ≥ 90). | OT += w["ot"]["closeness_gain"]×closeness − w["ot"]["decay"]×OT. |
| **bonding** | Nurturance bond. | bonding += w["bonding"]["ot_gain"]×OT − bonding_decay (decay from child_need or child_absent). |
| **fear_threat** | Threat proximity (child–threat distance). | fear += w["fear"]["threat_gain"]×threat_near − w["fear"]["decay"]×fear. |
| **CORT** (cortisol) | Stress hormone. | CORT += weighted(threat_near, child_need, energy_deficit) − w["cort"]["decay"]×CORT. |
| **stress** | Psychological stress. | stress += weighted(CORT, fear, child_need) − w["stress"]["decay"]×stress. |

**Child need** is a weighted mix of child hunger, warmth deviation, and injury, using **w["child_need"]**. It drives bonding decay (when child present), CORT, and stress.

So: **child proximity** → OT ↑ → **bonding** ↑; **threat** → **fear** ↑ and (via CORT) **stress** ↑; **child need** and **energy deficit** also feed CORT and stress.

---

## 3. Motivation

### 3.1 How motivations are computed

**motivation_compute(mother)** computes four scalar motivations (Forage, Care, Self, Protect) in [0, 100] using:

- **Deficits / signals** (normalized to ~[0,1]): mother energy deficit, closeness deficit, fear, bonding, fatigue, stress; child hunger, warmth, injury (or 0 if child dead).
- **Weights u** (see §4): for each motivation, a set of input weights (e.g. Forage: child_hunger, energy_deficit, low_fear).

Formula for each motivation **M** (e.g. Forage):

- **Inputs**: e.g. Forage uses `child_hunger`, `energy_deficit`, `low_fear` (low_fear = 1 − fear).
- **Weighted sum**:  
  `M = 100 × (u[mot][k1]×input1 + u[mot][k2]×input2 + ...) / sum(u[mot][k] for k in that motivation's keys)`  
  So it’s a **normalized weighted sum** of the inputs, scaled to [0, 100] and then clamped.

| Motivation | Inputs (and effect) |
|------------|----------------------|
| **Forage** | child_hunger, energy_deficit, low_fear |
| **Care**   | child_warmth, closeness_deficit, bonding |
| **Self**   | fatigue, fear, stress |
| **Protect**| child_injury, fear, closeness_deficit, bonding |

### 3.2 Selection and effect on behavior

- **select_motivation(mother)**: choose the motivation with **maximum** value:  
  `selected_motivation = argmax(motivations)`.
- **choose_goal_from_motivation(...)** then maps the selected motivation to a **goal** (e.g. food cell, child cell, safe cell) and optional **action** (e.g. care, eat, rest). Movement is planned toward that goal (pathfinding); if an action is chosen, the mother may stay and act instead of moving.

So: **states + u** → **four motivation values** → **argmax** → **one selected motivation** → **goal + action** → **move or act**.

---

## 4. Genes: fixed & plastic weights (self.w, self.u)

### 4.1 Two weight sets: w (psych) and u (motivation)

- **w**: weights for **psych/neuro** dynamics (child_need, OT, bonding, CORT, stress, fear). Used in **update_psych_neuro**.
- **u**: weights for **motivation** computation (forage, care, self, protect). Used in **motivation_compute**.

Each is split into **fixed** and **plastic**:

- **w_fixed**, **u_fixed**: set at birth (e.g. random in [0,1]), **never** changed in lifetime. “Genetic” baseline.
- **w_plastic**, **u_plastic**: **copy of fixed** at birth; can be **updated by plasticity** (see §5). Clamped to [0, 2].

### 4.2 How w and u are used (effective weight)

The agent never uses `w_fixed` / `u_fixed` or `w_plastic` / `u_plastic` alone. It uses the **average**:

- **self.w** = `(w_fixed + w_plastic) / 2`  (per key, per category)
- **self.u** = `(u_fixed + u_plastic) / 2`

So:

- **Psych/neuro** (OT, CORT, bonding, fear, stress, child_need) use **self.w**.
- **Motivation** computation uses **self.u**.

If plasticity is off, plastic equals fixed, so effective = fixed. If plasticity is on, learning changes **plastic**; **fixed** stays the same, so the **effective** policy is the average of inherited (fixed) and learned (plastic).

### 4.3 Structure (summary)

- **u_fixed / u_plastic**: categories `forage`, `care`, `self`, `protect`; each has keys like `child_hunger`, `energy_deficit`, `low_fear`, etc. Same keys in fixed and plastic.
- **w_fixed / w_plastic**: categories `child_need`, `ot`, `bonding`, `cort`, `stress`, `fear`; each has keys like `hunger`, `decay`, `threat_gain`, etc.

(Exact key names are in `core/agents.py` and `core/policies/mother.py`.)

---

## 5. Plasticity (two methods)

Plasticity only changes **u_plastic** and **w_plastic**. **u_fixed** and **w_fixed** never change.

### 5.1 Outcome-gated plasticity (**update_plasticity**)

- **When**: After the mother has acted (same tick). Uses **deficit before** (stored at decision time) and **deficit after** (current **overall_deficit(mother)**).
- **Signal**: Did things get better or worse?  
  - **deficit_after < deficit_before** → situation improved.  
  - **deficit_after > deficit_before** → situation worsened.
- **What is updated**: Only weights **relevant to the selected motivation**:
  - **u_plastic[selected_motivation][key]** for each key whose **input was active** (above a small threshold).
  - **w_plastic[cat][key]** for categories that “support” that motivation (see MOTIVATION_TO_W_CATEGORIES: forage→child_need,cort; care→bonding,ot; self→stress,cort,fear; protect→fear,child_need).
- **Rule**:
  - If **deficit decreased**: `u_plastic[M][key] += eta`, `w_plastic[cat][key] += eta`.
  - If **deficit increased**: `u_plastic[M][key] -= eta`, `w_plastic[cat][key] -= eta`.
- **Bounds**: Plastic weights clamped to [0, 2] (e.g. via _clamp_plastic).

So: **reward-like** learning from a scalar outcome (overall deficit). Improve → strengthen the policy that was used; worsen → weaken it.

### 5.2 Hebbian plasticity (**update_plasticity_hebbian**)

- **When**: Same as above (after action).
- **Signal**: **Co-activity** of “input” and “output”, not outcome:
  - **pre**: input activity (e.g. for u: the normalized deficit/state that feeds that motivation; for w: a scalar activity for that psych category).
  - **post**: how strongly the selected motivation fired (its value in [0,100], normalized to [0,1]).
- **Rule**:  
  `Δw = η × pre × post`  
  (and optionally subtract a small decay term pulling plastic toward fixed to avoid runaway growth).
- **What is updated**: Same as outcome-gated (u_plastic for selected motivation’s keys; w_plastic for MOTIVATION_TO_W_CATEGORIES for that motivation). All updated weights use the same **pre** and **post** for that tick.

So: **Hebbian** = “fire together, wire together” (strengthen when input and selected motivation were both active). No explicit good/bad outcome; optional decay keeps weights bounded.

---

## 6. How states and motivation affect each other (flow)

```
Child states (hunger, warmth, injury)
    ↓
Mother perceives child + threat + own energy, closeness, etc.
    ↓
update_psych_neuro(world)  [uses w]
    → child_need, OT, CORT, bonding, fear, stress, closeness_child
    ↓
motivation_compute(mother)  [uses u]
    → deficits from mother + child states
    → M_Forage, M_Care, M_Self, M_Protect = weighted sums of deficits
    ↓
select_motivation  → argmax(M_Forage, M_Care, M_Self, M_Protect)
    ↓
choose_goal_from_motivation  → goal cell + optional action (care, eat, rest, …)
    ↓
Movement + apply_mother_intents  → position change, feed/care/eat
    ↓
update_physiology(moved, acted)  → energy, fatigue
    ↓
(optional) Plasticity  → u_plastic, w_plastic  (using outcome or Hebbian rule)
```

- **w** shapes how **child and threat** drive **OT, CORT, bonding, fear, stress, closeness**.
- **u** shapes how **those psych states + child deficits** drive **each motivation**.
- **Selected motivation** drives **goal and action**; action and movement then change **energy/fatigue** and (via world) **child state** and **deficit**, which feed the next tick’s psych and motivation.

So the loop is: **world + child** → **psych (w)** → **motivation (u)** → **selection** → **goal/action** → **world/physiology** → (optional) **plasticity** → next tick.

---

## 7. File reference (where to look)

| What | Where |
|------|--------|
| World step order | `core/world.py` → `step()` |
| Physiology (energy, fatigue) | `core/agents.py` → `MotherAgent.update_physiology` |
| Psych/neuro (OT, CORT, bonding, fear, stress) | `core/agents.py` → `MotherAgent.update_psych_neuro` |
| Motivation computation & selection | `core/policies/mother.py` → `motivation_compute`, `select_motivation`, `choose_goal_from_motivation` |
| Ideal values & deficit helpers | `core/policies/deficit.py` |
| Fixed/plastic w, u and effective self.w, self.u | `core/agents.py` → `MotherAgent.__init__`, `self.w`, `self.u` |
| Outcome-gated plasticity | `core/policies/mother.py` → `update_plasticity` |
| Hebbian plasticity | `core/policies/mother.py` → `update_plasticity_hebbian` |
| Enabling plasticity in the world | `core/world.py` (plasticity_rule: `"outcome"` or `"hebbian"`) |

This README gives the overall pipeline; the code in these files contains the exact formulas and key names.
