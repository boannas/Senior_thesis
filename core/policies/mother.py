import numpy as np
from func.path_finding import astar
from core.sim.movement import in_bounds, best_step
from core.policies.deficit import IDEAL_VALUE, deficit_abs, deficit_high, deficit_low

np.random.seed(42)
# u_1 = [1, 1, 1]     # Foraging
# u_2 = [1, 1, 1]     # Care 
# u_3 = [1, 1, 1]    # Self-Preservation
# u_4 = [1, 1, 1, 1]     # Protect

u_1 = np.random.uniform(0,1 ,3)
u_2 = np.random.uniform(0,1 ,3)
u_3 = np.random.uniform(0,1 ,3)
u_4 = np.random.uniform(0,1 ,4)


def mother_policy_propose(world):
    perception_range = 100
    proposals = {}

    occupied_now = {(m.x, m.y) for m in world.mothers}
    mother_receive = world.foods + world.mothers + world.children + world.threats

    intended_food = {}    # mother -> Food
    forage_modes = {}
    intended_child = set()

    prev_pos = {m: (m.x, m.y) for m in world.mothers}

    for mother in world.mothers:

        if mother.fatigue >= 100:
            proposals[mother] = (mother.x, mother.y)
            continue

        food_perceived, _ = mother.scan_perception(mother_receive, perception_range=perception_range)

        goal = None
        child = mother.child

        # ----- Select motivation -----
        motivation_compute(mother)
        motivation = select_motivation(mother=mother)


        motivation = "Forage"
        goal, target_food, wants_child, forage_mode = choose_goal_from_motivation(world, mother, motivation, food_perceived)

        if wants_child:
            intended_child.add(mother)

        if target_food:
            intended_food[mother] = target_food

        if forage_mode is not None:
            forage_modes[mother] = forage_mode

        # print(goal, target_food, wants_child)

        


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
        "intended_food": intended_food,
        "intended_child": intended_child,
        "forage_modes": forage_modes
    }
    return proposals, intents

def apply_mother_intents(world, intents):
    prev_pos = intents["prev_pos"]
    intended_food = intents["intended_food"]
    intended_child = intents["intended_child"]
    forage_modes = intents["forage_modes"]

    # physiology
    for mother in world.mothers:
        moved = (prev_pos[mother] != (mother.x, mother.y))
        mother.update_physiology(moved=moved)

    # interactions
    for mother in world.mothers:
        # f = intended_food.get(mother)
        # if f is not None and (mother.x, mother.y) == (f.x, f.y) and not f.collected and mother.alive:
        #     f.collect()
        #     mother.picking_food(True)
            # mother.energy += 5

        f = intended_food.get(mother)        
        mode = forage_modes.get(mother)



        # print(mode)
        if f is not None and (mother.x, mother.y) == (f.x, f.y) and not f.collected and mother.alive:
            f.collect()
            mother.food_inventory += 1


        child = mother.child
        if mother in intended_child and child is not None and (mother.x, mother.y) == (child.x, child.y) and child.is_alive():
            mother.picking_child(True)
            child.set_carried(True)

        if child is not None and child.is_carried:
            child.x, child.y = mother.x, mother.y

def motivation_compute(mother):

    # Mother attr.
    m_energy_def = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])
    m_fear = mother.fear_threat
    m_bonding = mother.bonding
    m_closeness_def = deficit_abs(mother.closeness_child, IDEAL_VALUE['M_closeness'])
    m_fatigue = mother.fatigue
    m_stress = mother.stress

    if mother.child is not None:
        child = mother.child

        # Child deficit
        c_hunger_def = deficit_high(child.hunger, IDEAL_VALUE['C_hunger'])
        c_warmth_def = deficit_abs(child.warmth, IDEAL_VALUE['C_warmth'])
        c_injury_def = deficit_high(child.injury, IDEAL_VALUE['C_injury'])
    else:
        c_hunger_def = 0
        m_fear = 0
        c_warmth_def = 0
        m_closeness_def = 0
        c_injury_def = 0

    # Compute Motivation
    M_forage = (
        u_1[0] * c_hunger_def +
        u_1[1] * m_energy_def +
        u_1[2] * (1-m_fear)
    )

    M_Care = (
        u_2[0] * c_warmth_def + 
        u_2[1] * m_closeness_def +
        u_2[2] * m_bonding
    )


    M_self = (
        u_3[0] * m_fatigue +
        u_3[1] * m_fear +
        u_3[2] * m_stress
    )

    M_protect = (
        u_4[0] * c_injury_def + 
        u_4[1] * m_fear + 
        u_4[2] * m_closeness_def +
        u_4[3] * m_bonding
    )
    
    mother.motivations['Forage']    = clamp(M_forage)
    mother.motivations['Care']      = clamp(M_Care)
    mother.motivations['Self']      = clamp(M_self)
    mother.motivations['Protect']   = clamp(M_protect)

def select_motivation(mother):
    # for k, v in mother.motivations.items():
    #     print(f"{k}: {v:.2f}")

    motivation_names = list(mother.motivations.keys())
    motivation_values = list(mother.motivations.values())

    mot_idx = np.argmax(motivation_values)
    selected = motivation_names[mot_idx]
    # print(f"Selected motivation: {selected} ({motivation_values[mot_idx]:.2f})\n")

    return selected

def clamp(x, lo=0, hi=100):
    return max(lo, min(x, hi))


def choose_goal_from_motivation(world, mother, selected, food_perceived):
    child = mother.child


    if selected == "Forage":
        m_energy_def = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])

        if child is not None:
            c_hunger_def = deficit_high(child.hunger, IDEAL_VALUE['C_hunger'])
        else:
            c_hunger_def = 0
        
        print(mother.food_inventory)

        # ---- subdecision inside Forage ----
        if mother.food_inventory > 0:
            # already carrying food
            if child is not None and child.is_alive() and c_hunger_def > m_energy_def:
                return (child.x, child.y), None, False, "give_child"
            else:
                return (mother.x, mother.y), None, False, "eat_self"

        else:
            # no food yet -> go fetch food
            if food_perceived:
                food_perceived.sort(key=lambda t: t[1])
                target_food = food_perceived[0][0]

                if c_hunger_def > m_energy_def:
                    return (target_food.x, target_food.y), target_food, False, "fetch_for_child"
                else:
                    return (target_food.x, target_food.y), target_food, False, "fetch_for_self"

        return None, None, False, None
    
    

    elif selected == "Care":
        if child is not None and child.is_alive() and not child.is_carried:
            return (child.x, child.y), None, True, None
        return (mother.x, mother.y), None, False, None

    elif selected == "Protect":
        if child is not None and child.is_alive():
            return (child.x, child.y), None, False, None
        return (mother.x, mother.y), None, False, None

    elif selected == "Self":
        return (mother.x, mother.y), None, False, None

    return None, None, False, None