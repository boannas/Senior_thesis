import numpy as np
from func.path_finding import astar
from core.sim.movement import in_bounds, best_step
from core.policies.deficit import IDEAL_VALUE, deficit_abs, deficit_high, deficit_low

def mother_policy_propose(world):
    perception_range = 100
    proposals = {}

    occupied_now = {(m.x, m.y) for m in world.mothers}
    mother_receive = world.foods + world.mothers + world.children + world.threats

    intended_food = {}    # mother -> Food
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
        # if food_perceived and not mother.holding_food:
        if food_perceived:
            # Find food
            food_perceived.sort(key=lambda t: t[1])
            target_food = food_perceived[0][0]
            goal = (target_food.x, target_food.y)
            intended_food[mother] = target_food
        else:
            
            # if child is None or child.is_carried:
            #     proposals[mother] = (mother.x, mother.y)
            #     continue
            # if child.is_alive() :
            #     goal = (child.x, child.y)
            #     intended_child.add(mother)
            pass

        motivation_compute(mother)
        select_motivation(mother=mother)


        


        # ----- Hanndle Motivation not have -----
        if goal is None:
            valid_moves = []
            for dx, dy in world.RANDOM_MOVES:
                nx, ny = mother.x + dx, mother.y + dy
                if in_bounds(nx, ny, world.grid_w, world.grid_h):
                    valid_moves.append((nx, ny))
            # proposals[mother] = random.choice(valid_moves) if valid_moves else (mother.x, mother.y)
            proposals[mother] = (mother.x, mother.y)
            # print(proposals[mother])
            continue

        blocked = {(m.x, m.y) for m in world.mothers if m is not mother}
        blocked |= {(c.x, c.y) for c in world.children
                    if (c is not mother.child) and (not c.is_carried)}
        
        # block threats too 
        blocked |= {(t.x, t.y) for t in world.threats}

        path = astar((mother.x, mother.y), goal, world.grid_w, world.grid_h, blocked, moves_8=True)

        if path is None or len(path) < 2:
            nx, ny = best_step(mother, goal, occupied_now, world.grid_w, world.grid_h, world.RANDOM_MOVES)
        else:
            nx, ny = path[1]
            if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
                nx, ny = best_step(mother, goal, occupied_now, world.grid_w, world.grid_h, world.RANDOM_MOVES)
        proposals[mother] = (nx, ny)

    intents = {
        "prev_pos": prev_pos,
        "intended_food": intended_food,
        "intended_child": intended_child,
    }
    return proposals, intents

def apply_mother_intents(world, intents):
    prev_pos = intents["prev_pos"]
    intended_food = intents["intended_food"]
    intended_child = intents["intended_child"]

    # physiology
    for mother in world.mothers:
        moved = (prev_pos[mother] != (mother.x, mother.y))
        mother.update_physiology(moved=moved)

    # interactions
    for mother in world.mothers:
        f = intended_food.get(mother)
        if f is not None and (mother.x, mother.y) == (f.x, f.y) and not f.collected and mother.alive:
            f.collect()
            mother.picking_food(True)
            # mother.energy += 5

        child = mother.child
        if mother in intended_child and child is not None and (mother.x, mother.y) == (child.x, child.y) and child.is_alive():
            mother.picking_child(True)
            child.set_carried(True)

        if child is not None and child.is_carried:
            child.x, child.y = mother.x, mother.y



def motivation_compute(mother):
    u_1 = [1, 1, 1]     # Foraging
    u_2 = [1, 1, 1]     # Care 
    u_3 = [1.0, 1.0]    # Self-Preservation
    u_4 = [1,1,1,1]     # Protect


    # Mother attr.
    m_energy_def = deficit_low(mother.energy, IDEAL_VALUE['M_energy'])
    m_fear = mother.fear_threat
    m_bonding = mother.bonding
    m_closeness_def = deficit_abs(mother.closeness_child, IDEAL_VALUE['M_closeness'])
    m_fatigue = mother.fatigue

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
        # u_3[1] * m_energy_def
        u_3[1] * m_fear
    )

    M_protect = (
        u_4[0] * c_injury_def + 
        u_4[1] * m_fear + 
        u_4[2] * m_closeness_def +
        u_4[3] * m_bonding
    )

    motivation_names = ["forage", "care", "self", "protect"]
    motivation_arr = [M_forage, M_Care, M_self, M_protect]

    print(f"M_forage: {M_forage}\n",
          f"M_care: {M_Care}\n",
          f"M_self: {M_self}\n"
          f"M_protect: {M_protect}\n"
          )
    
    mother.motivations['Forage'] = M_forage
    mother.motivations['Care'] = M_Care
    mother.motivations['Self'] = M_self
    mother.motivations['Protect'] = M_protect



# def select_motivation(mother):
#     for k, v in mother.motivations.items():
#         print(f"{k}: {v:.2f}")
        
#     mot_idx = np.argmax(motivation_arr)
#     print('Selected motivation: ', motivation_names[mot_idx])


def select_motivation(mother):

    # print motivations
    for k, v in mother.motivations.items():
        print(f"{k}: {v:.2f}")

    motivation_names = list(mother.motivations.keys())
    motivation_values = list(mother.motivations.values())

    mot_idx = np.argmax(motivation_values)

    selected = motivation_names[mot_idx]

    print(f"Selected motivation: {selected} ({motivation_values[mot_idx]:.2f})")

    return selected