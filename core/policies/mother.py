import numpy as np
from func.path_finding import astar
from core.sim.movement import in_bounds, best_step
from core.policies.deficit import IDEAL_VALUE, deficit_abs, deficit_high, deficit_low
import random
random.seed(42)
np.random.seed(42)

def mother_policy_propose(world):
    perception_range = 100
    proposals = {}

    occupied_now = {(m.x, m.y) for m in world.mothers}
    mother_receive = world.foods + world.mothers + world.children + world.threats

    forage_modes = {}
    intended_actions = {}

    prev_pos = {m: (m.x, m.y) for m in world.mothers}

    for mother in world.mothers:

        if mother.fatigue >= 90:
            proposals[mother] = (mother.x, mother.y)
            continue

        food_perceived, _ = mother.scan_perception(mother_receive, perception_range=perception_range)

        goal = None
        child = mother.child

        # ----- Select motivation -----
        motivation_compute(mother)
        mother.selected_motivation = select_motivation(mother=mother)
        goal, action, forage_mode = choose_goal_from_motivation(
            world, mother,  mother.selected_motivation, food_perceived
            )
        
        print(goal, action, forage_mode)

        if action is not None:
            intended_actions[mother] = action
            proposals[mother] = (mother.x, mother.y)
            if forage_mode is not None:
                forage_modes[mother] = forage_mode
            continue

        if forage_mode is not None:
            forage_modes[mother] = forage_mode
        
        # ----- Hanndle Motivation not have -----
        if goal is None:
            valid_moves = []
            for dx, dy in world.RANDOM_MOVES:
                nx, ny = mother.x + dx, mother.y + dy
                if in_bounds(nx, ny, world.grid_w, world.grid_h):
                    valid_moves.append((nx, ny))
            # proposals[mother] = random.choice(valid_moves) if valid_moves else (mother.x, mother.y)
            proposals[mother] = (mother.x, mother.y)
            continue

        blocked = {(m.x, m.y) for m in world.mothers if m is not mother}
        blocked |= {(c.x, c.y) for c in world.children
                    if (c is not mother.child) and (not c.is_carried)}
        
        # block threats too 
        blocked |= {(t.x, t.y) for t in world.threats}


        if goal == (mother.x, mother.y):
            proposals[mother] = (mother.x, mother.y)
            continue


        path = astar((mother.x, mother.y), goal, world.grid_w, world.grid_h, blocked, moves_8=True)

        if path is None:
            nx, ny = best_step(mother, goal, occupied_now, world.grid_w, world.grid_h, world.RANDOM_MOVES)
        elif len(path) < 2:
            nx, ny = (mother.x, mother.y)   # already at goal, or degenerate path
        else:
            nx, ny = path[1]
            if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
                nx, ny = best_step(mother, goal, occupied_now, world.grid_w, world.grid_h, world.RANDOM_MOVES)

        
        proposals[mother] = (nx, ny)

    intents = {
        "prev_pos": prev_pos,
        "forage_modes": forage_modes,
        "intended_actions": intended_actions
    }
    return proposals, intents

def apply_mother_intents(world, intents):
    prev_pos = intents["prev_pos"]
    forage_modes = intents["forage_modes"]
    actions = intents["intended_actions"]

    # physiology
    for mother in world.mothers:
        moved = (prev_pos[mother] != (mother.x, mother.y))
        action = actions.get(mother)
        acted = action is not None
        mother.update_physiology(moved=moved, acted=acted)

    for mother in world.mothers:
        child = mother.child
        if child is None:
            continue
        moved = prev_pos[mother] != (mother.x, mother.y)
        if child.is_carried and moved:
            child.set_carried(False)
            mother.picking_child(False)
            child.x, child.y = prev_pos[mother]

    # Action
    for mother in world.mothers:
        action = actions.get(mother)
        child = mother.child


        if action == "care":
            if child and (mother.x, mother.y) == (child.x, child.y):
                child.warmth = min(50.0, child.warmth + 1.0)
        
        elif action == "rest":
            pass

        elif action == 'eat':
            if mother.food_inventory > 0:
                mother.food_inventory -= 1
                mother.energy = min(100, mother.energy + 5)
            else: 
                f = food_at_cell(world, mother.x, mother.y)
                if f is not None:
                    f.collect()
                    mother.energy = min(100, mother.energy + 5)
        
        elif action == 'pick_food':
            f = food_at_cell(world, mother.x, mother.y)
            if f and not f.collected:
                f.collect()
                mother.food_inventory += 1

        elif action == "feed_child":
            if child and child.is_alive() and mother.food_inventory > 0 and (mother.x, mother.y) == (child.x, child.y):
                mother.food_inventory -= 1
                child.hunger = max(0, child.hunger - 20)
            
        elif action == 'threaten':
            pass


def overall_deficit(mother):
    """
    Scalar overall deficit = all mother deficits + all child deficits (normalized 0–1).
    Used for outcome-gated plasticity: did the situation get better or worse after acting?
    Mother: energy, closeness, fear, stress, fatigue, bonding (all vs IDEAL_VALUE).
    Child: hunger, warmth, injury (all vs IDEAL_VALUE).
    """
    n = 100.0
    # Mother deficits (ideal: high energy, mid closeness, zero fear/stress/fatigue, high bonding)
    m_energy_def = deficit_low(mother.energy, IDEAL_VALUE["M_energy"]) / n
    m_closeness_def = deficit_abs(mother.closeness_child, IDEAL_VALUE["M_closeness"]) / n
    m_fear_def = deficit_high(mother.fear_threat, IDEAL_VALUE["M_fear"]) / n
    m_stress_def = deficit_high(mother.stress, IDEAL_VALUE["M_stress"]) / n
    m_fatigue_def = deficit_high(mother.fatigue, IDEAL_VALUE["M_fatigue"]) / n
    m_bonding_def = deficit_low(mother.bonding, IDEAL_VALUE["M_bonding"]) / n
    total = m_energy_def + m_closeness_def + m_fear_def + m_stress_def + m_fatigue_def + m_bonding_def
    # Child deficits (ideal: zero hunger, mid warmth, zero injury)
    if mother.child is not None and mother.child.is_alive():
        c = mother.child
        total += deficit_high(c.hunger, IDEAL_VALUE["C_hunger"]) / n
        total += deficit_abs(c.warmth, IDEAL_VALUE["C_warmth"]) * 2 / n  # warmth in [0,50]
        total += deficit_high(c.injury, IDEAL_VALUE["C_injury"]) / n
    return total


def _clamp_plastic(plastic_dict, cat, key, lo=0.0, hi=2.0):
    plastic_dict[cat][key] = max(lo, min(hi, plastic_dict[cat][key]))


def _motivation_to_u_key(mot):
    return mot.lower() if mot else None


# Which w (psych) categories to update when this motivation was selected (support that motivation)
MOTIVATION_TO_W_CATEGORIES = {
    "forage": ["child_need", "cort"],
    "care": ["bonding", "ot"],
    "self": ["stress", "cort", "fear"],
    "protect": ["fear", "child_need"],
}


def update_plasticity(mother, action, world):
    """
    Outcome-gated plasticity for both u (motivation weights) and w (psych weights):
    - If overall deficit decreased → strengthen plastic weights that were active for selected motivation.
    - If overall deficit increased → weaken them (learn it may not be best in that situation).
    """
    if not hasattr(mother, "u_plastic"):
        return
    eta = getattr(mother, "eta", 0.02)
    u_plastic = mother.u_plastic
    deficit_before = getattr(mother, "_deficit_before", None)
    if deficit_before is None:
        return
    deficit_after = overall_deficit(mother)
    inputs = getattr(mother, "_last_motivation_inputs", None)
    if inputs is None:
        return
    M = _motivation_to_u_key(mother.selected_motivation)
    if M is None or M not in u_plastic:
        return
    state_threshold = 0.25
    # --- Plasticity for u (motivation weights) ---
    for key in u_plastic[M]:
        val = inputs.get(M, {}).get(key, 0.0)
        if val < state_threshold:
            continue
        if deficit_after < deficit_before:
            u_plastic[M][key] += eta
        elif deficit_after > deficit_before:
            u_plastic[M][key] -= eta
        _clamp_plastic(u_plastic, M, key)
    # --- Plasticity for w (psych weights): same outcome gating, for categories that support M ---
    if hasattr(mother, "w_plastic"):
        w_plastic = mother.w_plastic
        categories = MOTIVATION_TO_W_CATEGORIES.get(M, [])
        for cat in categories:
            if cat not in w_plastic:
                continue
            for key in w_plastic[cat]:
                if deficit_after < deficit_before:
                    w_plastic[cat][key] += eta
                elif deficit_after > deficit_before:
                    w_plastic[cat][key] -= eta
                _clamp_plastic(w_plastic, cat, key)


def motivation_compute(mother):

    # Mother attr.
    m_closeness_def = deficit_abs(mother.closeness_child, IDEAL_VALUE['M_closeness'])
    m_energy_def = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])
    m_fear = mother.fear_threat
    m_bonding = mother.bonding
    m_fatigue = mother.fatigue
    m_stress = mother.stress

    # Normalize Mother attr.
    normalize_value = 100.0
    m_closeness_def = m_closeness_def / normalize_value
    m_energy_def = m_energy_def / normalize_value
    m_fear = m_fear / normalize_value
    m_bonding = m_bonding / normalize_value
    m_fatigue = m_fatigue / normalize_value
    m_stress = m_stress / normalize_value

    # Child alive >,<
    if mother.child is not None and mother.child.alive:
        child = mother.child
        # Child deficit
        c_hunger_def = deficit_high(child.hunger, IDEAL_VALUE['C_hunger'])
        c_warmth_def = deficit_abs(child.warmth, IDEAL_VALUE['C_warmth']) * 2 # range is for [0,50]
        c_injury_def = deficit_high(child.injury, IDEAL_VALUE['C_injury'])

        # Normalize Child deficit
        c_hunger_def /= normalize_value
        c_warmth_def /= normalize_value
        c_injury_def /= normalize_value
        # print(c_hunger_def)
    
    # Child died T_T
    else:
        c_hunger_def = 0
        c_warmth_def = 0
        m_closeness_def = 0
        c_injury_def = 0

    # --- Compute Motivation ---
    # Forage
    forage_u = mother.u["forage"]
    forage_u_sum = weight_sum(
        forage_u,
        ['child_hunger', 'energy_deficit', 'low_fear']
    )
    M_forage = 100.0 * (
        forage_u['child_hunger'] * c_hunger_def +
        forage_u['energy_deficit'] * m_energy_def +
        forage_u['low_fear'] * (1 - m_fear)
    ) / forage_u_sum

    # Care
    care_u = mother.u["care"]
    care_u_sum = weight_sum(
        care_u,
        ['child_warmth', 'closeness_deficit', 'bonding']
    )
    M_Care = 100.0 * (
        care_u['child_warmth'] * c_warmth_def + 
        care_u['closeness_deficit'] * m_closeness_def +
        care_u['bonding'] * m_bonding
    ) / care_u_sum


    # Self
    self_u = mother.u["self"]
    self_u_sum = weight_sum(
        self_u,
        ['fatigue', 'fear', 'stress']
    )
    M_self = 100.0 * (
        self_u['fatigue'] * m_fatigue +
        self_u['fear'] * m_fear +
        self_u['stress'] * m_stress
    ) / self_u_sum


    # Protect
    protect_u = mother.u["protect"]
    protect_u_sum = weight_sum(
        protect_u,
        ['child_injury', 'fear', 'closeness_deficit', 'bonding']
    )
    M_protect = 100.0 * (
        protect_u['child_injury'] * c_injury_def + 
        protect_u['fear'] * m_fear + 
        protect_u['closeness_deficit'] * m_closeness_def +
        protect_u['bonding'] * m_bonding
    ) / protect_u_sum
    
    mother.motivations['Forage']    = clamp(M_forage)
    mother.motivations['Care']      = clamp(M_Care)
    mother.motivations['Self']      = clamp(M_self)
    mother.motivations['Protect']   = clamp(M_protect)

    # For outcome-gated plasticity: store deficit and inputs at decision time
    mother._last_motivation_inputs = {
        "forage": {"child_hunger": c_hunger_def, "energy_deficit": m_energy_def, "low_fear": 1.0 - m_fear},
        "care": {"child_warmth": c_warmth_def, "closeness_deficit": m_closeness_def, "bonding": m_bonding},
        "self": {"fatigue": m_fatigue, "fear": m_fear, "stress": m_stress},
        "protect": {"child_injury": c_injury_def, "fear": m_fear, "closeness_deficit": m_closeness_def, "bonding": m_bonding},
    }
    mother._deficit_before = overall_deficit(mother)

def select_motivation(mother):
    # for k, v in mother.motivations.items():
    #     print(f"{k}: {v:.2f}")

    motivation_names = list(mother.motivations.keys())
    motivation_values = list(mother.motivations.values())

    mot_idx = np.argmax(motivation_values)
    selected = motivation_names[mot_idx]
    print(f"Selected motivation: {selected} ({motivation_values[mot_idx]:.2f})\n")

    return selected

def clamp(x, lo=0.0, hi=100.0):
    return max(lo, min(x, hi))

def choose_goal_from_motivation(world, mother, selected, food_perceived):
    child = mother.child


    if selected == "Forage":

        m_energy_def = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])
        c_hunger_def = deficit_high(child.hunger, IDEAL_VALUE['C_hunger']) if child else 0
        
        # carring food already
        if mother.food_inventory > 0:
            if child and c_hunger_def > m_energy_def:
                if (mother.x, mother.y) == (child.x, child.y):
                    return (mother.x, mother.y), "feed_child", "give_child"
                else:
                    return (child.x, child.y), None, "give_child"
                
            else: 
                return (mother.x, mother.y), "eat", "eat_self"
            
        # not carrying food 
        if food_perceived:
            food_perceived.sort(key=lambda t:t[1])
            target_food = food_perceived[0][0]

            if (mother.x, mother.y) == (target_food.x, target_food.y):
                if child and c_hunger_def > m_energy_def:
                    return (mother.x, mother.y), "pick_food", "fetch_for_child"
                else: 
                    return (mother.x, mother.y), "eat", "fetch_for_self"
                
            return (target_food.x, target_food.y), None, None
        return (mother.x, mother.y), None, None    

    elif selected == "Care":
        if child is not None and child.is_alive():
            if (mother.x, mother.y) == (child.x, child.y):
                return (mother.x, mother.y), "care", None
            else:
                return (child.x, child.y), None, None

    elif selected == "Protect":
        if child is None or not child.is_alive():
            return (mother.x, mother.y), None, None
        
        d_child = mother.distance_to(child.x, child.y)

        if d_child > 1:
            return (child.x, child.y), None, None
        
        # Find nearest threat
        nearest = None
        best_d = 999
        for t in world.threats:
            d = child.distance_to(t.x, t.y)
            if d < best_d:
                best_d = d
                nearest = t

        if nearest is not None:
            return (nearest.x, nearest.y), None, None
        
        return (mother.x, mother.y), None, None


    elif selected == "Self":
        return (mother.x, mother.y), "rest", None

    return None, None, None



def weight_sum(group, keys):
    return sum(group[k] for k in keys)

def food_at_cell(world, x, y):
    for f in world.foods:
        if not f.collected and (f.x, f.y) == (x, y):
            return f
    return None