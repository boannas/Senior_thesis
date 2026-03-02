import random
from func.path_finding import astar
from core.sim.movement import in_bounds, best_step

def mother_policy_propose(world):
    perception_range = 100
    proposals = {}

    occupied_now = {(m.x, m.y) for m in world.mothers}
    mother_receive = world.foods + world.mothers + world.children + world.threats

    intended_food = {}    # mother -> Food
    intended_child = set()

    prev_pos = {m: (m.x, m.y) for m in world.mothers}

    for mother in world.mothers:
        food_perceived, _ = mother.scan_perception(mother_receive, perception_range=perception_range)

        goal = None
        child = mother.child

        if food_perceived and not mother.holding_food:
            food_perceived.sort(key=lambda t: t[1])
            target_food = food_perceived[0][0]
            goal = (target_food.x, target_food.y)
            intended_food[mother] = target_food
        else:
            if child is None or child.is_carried:
                proposals[mother] = (mother.x, mother.y)
                continue
            if child.is_alive():
                goal = (child.x, child.y)
                intended_child.add(mother)

        if goal is None:
            valid_moves = []
            for dx, dy in world.RANDOM_MOVES:
                nx, ny = mother.x + dx, mother.y + dy
                if in_bounds(nx, ny, world.grid_w, world.grid_h):
                    valid_moves.append((nx, ny))
            proposals[mother] = random.choice(valid_moves) if valid_moves else (mother.x, mother.y)
            continue

        blocked = {(m.x, m.y) for m in world.mothers if m is not mother}
        blocked |= {(c.x, c.y) for c in world.children
                    if (c is not mother.child) and (not c.is_carried)}
        # optionally block threats too (recommended)
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
        if f is not None and (mother.x, mother.y) == (f.x, f.y) and not f.collected:
            f.collect()
            mother.picking_food(True)

        child = mother.child
        if mother in intended_child and child is not None and (mother.x, mother.y) == (child.x, child.y) and child.is_alive():
            child.set_carried(True)

        if child is not None and child.is_carried:
            child.x, child.y = mother.x, mother.y