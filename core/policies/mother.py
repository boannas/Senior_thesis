"""
Mother agent decision-making policy.

Pipeline per tick:
1. Compute motivation scores (Forage, Care, Self, Protect)
2. Select dominant motivation (argmax)
3. Choose goal position + immediate action from motivation
4. Pathfind toward goal (A* or greedy fallback)
5. Apply actions (feed, care, eat, rest, protect)
6. Update outcome-gated plasticity (learn from results)
"""

import numpy as np
from func.path_finding import astar
from core.agents import weighted_sum
from core.sim.movement import in_bounds, best_step
from core.policies.deficit import IDEAL_VALUE, deficit_abs, deficit_high, deficit_low


# ─── Helpers ────────────────────────────────────────────────────────────


def food_at_cell(world, x, y):
    """Return the first uncollected food at (x, y), or None."""
    for food in world.foods:
        if not food.collected and (food.x, food.y) == (x, y):
            return food
    return None


def clamp(value, lo=0.0, hi=100.0):
    """Clamp a value to [lo, hi]."""
    return max(lo, min(value, hi))


# ═══════════════════════════════════════════════════════════════════════
# STEP 1 — Proposal Generation (movement + action intent)
# ═══════════════════════════════════════════════════════════════════════

def mother_policy_propose(world):
    """
    For each mother, compute motivation → choose goal → plan path.

    Returns
    -------
    proposals : dict {mother: (target_x, target_y)}
    intents   : dict with 'prev_pos', 'forage_modes', 'intended_actions'
    """
    perception_range = 100
    proposals = {}
    occupied_now = {(m.x, m.y) for m in world.mothers}
    all_visible_entities = world.foods + world.mothers + world.children + world.threats

    forage_modes = {}
    intended_actions = {}
    prev_positions = {mother: (mother.x, mother.y) for mother in world.mothers}

    for mother in world.mothers:
        food_perceived, _ = mother.scan_perception(
            all_visible_entities, perception_range=perception_range
        )

        # --- Motivation selection ---
        compute_motivations(mother)
        mother.selected_motivation = select_motivation(mother)
        goal, action, forage_mode = choose_goal_from_motivation(
            world, mother, mother.selected_motivation, food_perceived
        )


        # If an immediate action was chosen (no movement needed)
        if action is not None:
            intended_actions[mother] = action
            proposals[mother] = (mother.x, mother.y)
            if forage_mode is not None:
                forage_modes[mother] = forage_mode
            continue

        if forage_mode is not None:
            forage_modes[mother] = forage_mode

        # No goal → stay put
        if goal is None:
            proposals[mother] = (mother.x, mother.y)
            continue

        # Already at goal → stay
        if goal == (mother.x, mother.y):
            proposals[mother] = (mother.x, mother.y)
            continue

        # --- Pathfinding ---
        blocked = {(m.x, m.y) for m in world.mothers if m is not mother}
        blocked |= {(c.x, c.y) for c in world.children
                    if (c is not mother.child) and (not c.is_carried)}
        blocked |= {(t.x, t.y) for t in world.threats}

        path = astar(
            (mother.x, mother.y), goal,
            world.grid_w, world.grid_h, blocked, moves_8=True
        )

        if path is None:
            next_x, next_y = best_step(
                mother, goal, occupied_now,
                world.grid_w, world.grid_h, world.RANDOM_MOVES
            )
        elif len(path) < 2:
            next_x, next_y = mother.x, mother.y
        else:
            next_x, next_y = path[1]
            if (next_x, next_y) in occupied_now and (next_x, next_y) != (mother.x, mother.y):
                next_x, next_y = best_step(
                    mother, goal, occupied_now,
                    world.grid_w, world.grid_h, world.RANDOM_MOVES
                )

        proposals[mother] = (next_x, next_y)

    intents = {
        "prev_pos": prev_positions,
        "forage_modes": forage_modes,
        "intended_actions": intended_actions,
    }
    return proposals, intents


# ═══════════════════════════════════════════════════════════════════════
# STEP 2 — Apply Actions
# ═══════════════════════════════════════════════════════════════════════

def apply_mother_intents(world, intents):
    """Apply all mother actions (move costs, child drop, feed, care, etc.)."""
    prev_positions = intents["prev_pos"]
    actions = intents["intended_actions"]

    # --- Update physiology (energy/fatigue costs) ---
    for mother in world.mothers:
        moved = (prev_positions[mother] != (mother.x, mother.y))
        action = actions.get(mother)
        acted = action is not None and action != "rest"
        rested = (action == "rest")
        mother.update_physiology(moved=moved, acted=acted, rested=rested)

    # --- Drop child if mother moved while carrying ---
    for mother in world.mothers:
        child = mother.child
        if child is None:
            continue
        moved = prev_positions[mother] != (mother.x, mother.y)
        if child.is_carried and moved:
            child.set_carried(False)
            mother.picking_child(False)
            child.x, child.y = prev_positions[mother]

    # --- Execute actions ---
    for mother in world.mothers:
        action = actions.get(mother)
        child = mother.child

        if action == "care":
            if child and (mother.x, mother.y) == (child.x, child.y):
                child.warmth = min(50.0, child.warmth + 1.0)

        elif action == "rest":
            pass  # Mother rests (no action needed, physiology handles recovery)

        elif action == "eat":
            if mother.food_inventory > 0:
                mother.food_inventory -= 1
                mother.energy = min(100, mother.energy + 5)
            else:
                food = food_at_cell(world, mother.x, mother.y)
                if food is not None:
                    food.collect()
                    mother.energy = min(100, mother.energy + 5)

        elif action == "pick_food":
            food = food_at_cell(world, mother.x, mother.y)
            if food and not food.collected:
                food.collect()
                mother.food_inventory += 1

        elif action == "feed_child":
            if (child and child.is_alive()
                    and mother.food_inventory > 0
                    and (mother.x, mother.y) == (child.x, child.y)):
                mother.food_inventory -= 1
                child.hunger = max(0, child.hunger - 20)

        elif action == "threaten":
            pass  # Placeholder for future threat confrontation mechanics

    # --- Step 6: plasticity update (optional, controlled by world.plasticity_rule) ---
    plasticity_rule = getattr(world, "plasticity_rule", "outcome")
    if plasticity_rule is None:
        return
    if plasticity_rule != "outcome":
        raise NotImplementedError(f"Unsupported plasticity_rule on baseline: {plasticity_rule!r}")
    for mother in world.mothers:
        update_plasticity(mother, actions.get(mother), world)


# ═══════════════════════════════════════════════════════════════════════
# Motivation Computation
# ═══════════════════════════════════════════════════════════════════════

def compute_motivations(mother):
    """
    Compute all four motivation scores for a mother based on her
    current states and her child's needs.

    Motivations:
    - Forage:  driven by child hunger, own energy deficit, low fear
    - Care:    driven by child warmth deficit, closeness deficit, bonding
    - Self:    driven by fatigue, fear, stress
    - Protect: driven by child injury, fear, closeness deficit, bonding
    """
    normalize = 100.0
    mw = mother.motivation_weights  # shorthand

    # --- Compute mother-side signals (normalized to [0, 1]) ---
    closeness_deficit = deficit_abs(mother.closeness_child, IDEAL_VALUE['M_closeness']) / normalize
    energy_deficit = deficit_low(mother.energy, IDEAL_VALUE['M_energy']) / normalize
    fear = mother.fear_threat / normalize
    bonding = mother.bonding / normalize
    fatigue = mother.fatigue / normalize
    stress = mother.stress / normalize

    # --- Compute child-side signals ---
    if mother.child is not None and mother.child.alive:
        child = mother.child
        child_hunger = deficit_high(child.hunger, IDEAL_VALUE['C_hunger']) / normalize
        child_warmth = deficit_abs(child.warmth, IDEAL_VALUE['C_warmth']) * 2 / normalize  # range [0,50]
        child_injury = deficit_high(child.injury, IDEAL_VALUE['C_injury']) / normalize
    else:
        child_hunger = 0.0
        child_warmth = 0.0
        child_injury = 0.0
        closeness_deficit = 0.0

    # --- Forage motivation ---
    forage_sum = weighted_sum(mw["forage"], ['child_hunger', 'energy_deficit', 'low_fear'])
    motivation_forage = 100.0 * (
        mw["forage"]["child_hunger"] * child_hunger +
        mw["forage"]["energy_deficit"] * energy_deficit +
        mw["forage"]["low_fear"] * (1.0 - fear)
    ) / forage_sum

    # --- Care motivation ---
    care_sum = weighted_sum(mw["care"], ['child_warmth', 'closeness_deficit', 'bonding'])
    motivation_care = 100.0 * (
        mw["care"]["child_warmth"] * child_warmth +
        mw["care"]["closeness_deficit"] * closeness_deficit +
        mw["care"]["bonding"] * bonding
    ) / care_sum

    # --- Self motivation ---
    self_sum = weighted_sum(mw["self"], ['fatigue', 'fear', 'stress'])
    motivation_self = 100.0 * (
        mw["self"]["fatigue"] * fatigue +
        mw["self"]["fear"] * fear +
        mw["self"]["stress"] * stress
    ) / self_sum

    # --- Protect motivation ---
    # Protect only activates when threats are nearby (fear > 0)
    # Without threats, protect should be near-zero to avoid dominating peaceful behavior
    has_threat_signal = fear > 0.01 or child_injury > 0.01
    if has_threat_signal:
        protect_sum = weighted_sum(mw["protect"], ['child_injury', 'fear', 'closeness_deficit', 'bonding'])
        motivation_protect = 100.0 * (
            mw["protect"]["child_injury"] * child_injury +
            mw["protect"]["fear"] * fear +
            mw["protect"]["closeness_deficit"] * closeness_deficit +
            mw["protect"]["bonding"] * bonding
        ) / protect_sum
    else:
        motivation_protect = 0.0

    # --- Store results ---
    mother.motivations['Forage'] = clamp(motivation_forage)
    mother.motivations['Care'] = clamp(motivation_care)
    mother.motivations['Self'] = clamp(motivation_self)
    mother.motivations['Protect'] = clamp(motivation_protect)

    # --- Store inputs for plasticity learning (used after action) ---
    mother._last_motivation_inputs = {
        "forage":  {"child_hunger": child_hunger, "energy_deficit": energy_deficit, "low_fear": 1.0 - fear},
        "care":    {"child_warmth": child_warmth, "closeness_deficit": closeness_deficit, "bonding": bonding},
        "self":    {"fatigue": fatigue, "fear": fear, "stress": stress},
        "protect": {"child_injury": child_injury, "fear": fear, "closeness_deficit": closeness_deficit, "bonding": bonding},
    }
    mother._deficit_before = compute_overall_deficit(mother)


def select_motivation(mother):
    """Select the highest-scoring motivation as the dominant one."""
    motivation_names = list(mother.motivations.keys())
    motivation_values = list(mother.motivations.values())
    best_index = np.argmax(motivation_values)
    return motivation_names[best_index]


# ═══════════════════════════════════════════════════════════════════════
# Goal Selection from Motivation
# ═══════════════════════════════════════════════════════════════════════

def choose_goal_from_motivation(world, mother, selected_motivation, food_perceived):
    """
    Given the selected motivation, determine:
    - goal: (x, y) position to move toward
    - action: immediate action to perform (or None)
    - forage_mode: sub-mode for forage motivation (or None)
    """
    child = mother.child

    if selected_motivation == "Forage":
        return _goal_forage(world, mother, child, food_perceived)

    elif selected_motivation == "Care":
        if child is not None and child.is_alive():
            if (mother.x, mother.y) == (child.x, child.y):
                return (mother.x, mother.y), "care", None
            else:
                return (child.x, child.y), None, None

    elif selected_motivation == "Protect":
        return _goal_protect(world, mother, child)

    elif selected_motivation == "Self":
        return (mother.x, mother.y), "rest", None

    return None, None, None


def _goal_forage(world, mother, child, food_perceived):
    """Determine forage goal: eat self, feed child, or search for food."""
    energy_deficit = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])
    child_hunger = deficit_high(child.hunger, IDEAL_VALUE['C_hunger']) if child else 0

    # Already carrying food → decide who to feed
    if mother.food_inventory > 0:
        if child and child_hunger > energy_deficit:
            if (mother.x, mother.y) == (child.x, child.y):
                return (mother.x, mother.y), "feed_child", "give_child"
            else:
                return (child.x, child.y), None, "give_child"
        else:
            return (mother.x, mother.y), "eat", "eat_self"

    # Not carrying food → find nearest food
    if food_perceived:
        food_perceived.sort(key=lambda item: item[1])
        target_food = food_perceived[0][0]

        if (mother.x, mother.y) == (target_food.x, target_food.y):
            if child and child_hunger > energy_deficit:
                return (mother.x, mother.y), "pick_food", "fetch_for_child"
            else:
                return (mother.x, mother.y), "eat", "fetch_for_self"

        return (target_food.x, target_food.y), None, None

    return (mother.x, mother.y), None, None


def _goal_protect(world, mother, child):
    """Determine protect goal: move to child, then confront nearest threat."""
    if child is None or not child.is_alive():
        return (mother.x, mother.y), None, None

    distance_to_child = mother.distance_to(child.x, child.y)

    # First priority: get close to child
    if distance_to_child > 1:
        return (child.x, child.y), None, None

    # Already near child → approach nearest threat
    nearest_threat = None
    best_dist = 999
    for threat in world.threats:
        dist = child.distance_to(threat.x, threat.y)
        if dist < best_dist:
            best_dist = dist
            nearest_threat = threat

    if nearest_threat is not None:
        return (nearest_threat.x, nearest_threat.y), None, None

    return (mother.x, mother.y), None, None


# ═══════════════════════════════════════════════════════════════════════
# Overall Deficit (for plasticity learning)
# ═══════════════════════════════════════════════════════════════════════

def compute_overall_deficit(mother):
    """
    Compute a scalar overall deficit score (0 = everything ideal).
    Used for outcome-gated plasticity: did the situation improve after acting?
    """
    norm = 100.0

    # Mother deficits
    total = (
        deficit_low(mother.energy, IDEAL_VALUE["M_energy"]) / norm +
        deficit_abs(mother.closeness_child, IDEAL_VALUE["M_closeness"]) / norm +
        deficit_high(mother.fear_threat, IDEAL_VALUE["M_fear"]) / norm +
        deficit_high(mother.stress, IDEAL_VALUE["M_stress"]) / norm +
        deficit_high(mother.fatigue, IDEAL_VALUE["M_fatigue"]) / norm +
        deficit_low(mother.bonding, IDEAL_VALUE["M_bonding"]) / norm
    )

    # Child deficits
    if mother.child is not None and mother.child.is_alive():
        child = mother.child
        total += deficit_high(child.hunger, IDEAL_VALUE["C_hunger"]) / norm
        total += deficit_abs(child.warmth, IDEAL_VALUE["C_warmth"]) * 2 / norm
        total += deficit_high(child.injury, IDEAL_VALUE["C_injury"]) / norm

    return total


# Backwards-compatible alias (used by mother-plasticity experiments)
def overall_deficit(mother):
    return compute_overall_deficit(mother)


# ═══════════════════════════════════════════════════════════════════════
# Outcome-Gated Plasticity (Learning)
# ═══════════════════════════════════════════════════════════════════════

# Which psych weight categories support each motivation
MOTIVATION_TO_PSYCH_CATEGORIES = {
    "forage":  ["child_need", "cortisol"],
    "care":    ["bonding", "oxytocin"],
    "self":    ["stress", "cortisol", "fear"],
    "protect": ["fear", "child_need"],
}


def _clamp_plastic_weight(plastic_dict, category, key, lo=0.0, hi=2.0):
    """Clamp a single plastic weight to [lo, hi]."""
    plastic_dict[category][key] = max(lo, min(hi, plastic_dict[category][key]))


def update_plasticity(mother, action, world):
    """
    Outcome-gated Hebbian learning:
    - If overall deficit DECREASED after acting → strengthen active weights
    - If overall deficit INCREASED after acting → weaken active weights

    Updates both motivation weights (u) and psych weights (w).
    """
    if not hasattr(mother, "motivation_weights_plastic"):
        return

    learning_rate = getattr(mother, "learning_rate", 0.02)
    deficit_before = getattr(mother, "_deficit_before", None)
    if deficit_before is None:
        return

    deficit_after = compute_overall_deficit(mother)
    inputs = getattr(mother, "_last_motivation_inputs", None)
    if inputs is None:
        return

    selected = mother.selected_motivation.lower() if mother.selected_motivation else None
    if selected is None or selected not in mother.motivation_weights_plastic:
        return

    state_threshold = 0.25

    # --- Update motivation weights (plastic) ---
    mot_plastic = mother.motivation_weights_plastic
    for key in mot_plastic[selected]:
        input_value = inputs.get(selected, {}).get(key, 0.0)
        if input_value < state_threshold:
            continue
        if deficit_after < deficit_before:
            mot_plastic[selected][key] += learning_rate
        elif deficit_after > deficit_before:
            mot_plastic[selected][key] -= learning_rate
        _clamp_plastic_weight(mot_plastic, selected, key)

    # --- Update psych weights (plastic) ---
    if hasattr(mother, "psych_weights_plastic"):
        psych_plastic = mother.psych_weights_plastic
        relevant_categories = MOTIVATION_TO_PSYCH_CATEGORIES.get(selected, [])
        for category in relevant_categories:
            if category not in psych_plastic:
                continue
            for key in psych_plastic[category]:
                if deficit_after < deficit_before:
                    psych_plastic[category][key] += learning_rate
                elif deficit_after > deficit_before:
                    psych_plastic[category][key] -= learning_rate
                _clamp_plastic_weight(psych_plastic, category, key)